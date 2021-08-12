////////////////////////////////////////////////////////////////////////////////
/// atsc-cc-extractor
///
/// Program that outputs embedded CEA-708 closed captions in SRT format from
/// over-the-air ATSC 1.0 TV recordings captured in MPEG-TS format.
///
/// References
/// - http://www.bretl.com/mpeghtml/vidstruc.HTM
/// - http://dvd.sourceforge.net/dvdinfo/mpeghdrs.html
/// - http://eilat.sci.brooklyn.cuny.edu/cis52/userfiles/file/MPEG.pdf
/// - http://www.cs.columbia.edu/~delbert/docs/Dueck%20--%20MPEG-2%20Video%20Transcoding.pdf
/// - https://comcast.github.io/caption-inspector/html/docs-page.html

#include "utils.hpp"
#include "mpegts.hpp"

static constexpr uint32_t START_CODE_SEQ = 0x000001B3;
static constexpr uint32_t START_CODE_EXT = 0x000001B5;
static constexpr uint32_t START_CODE_GOP = 0x000001B8;
static constexpr uint32_t START_CODE_PIC = 0x00000100;
static constexpr uint32_t START_CODE_USR = 0x000001B2;
static constexpr uint32_t START_CODE_SLL = 0x00000101;
static constexpr uint32_t START_CODE_SLH = 0x000001AF;

static bool depacketize_stream(const std::vector<utils::StreamBuffer>& packets, utils::StreamBuffer& out);
static bool extract_dtvcc_packets(const utils::StreamBuffer& mpeg2_stream, std::vector<utils::StreamBuffer>& out);
static bool parse_mpeg2_sequence(std::basic_string_view<uint8_t>& iter, std::vector<utils::StreamBuffer>& out);
static bool parse_mpeg2_gop(std::basic_string_view<uint8_t>& iter, std::vector<utils::StreamBuffer>& out);
static bool parse_mpeg2_picture(std::basic_string_view<uint8_t>& iter, std::vector<utils::StreamBuffer>& out);
static bool parse_mpeg2_slice(std::basic_string_view<uint8_t>& iter);
static bool parse_mpeg2_user(std::basic_string_view<uint8_t>& iter, std::vector<utils::StreamBuffer>& out);

int main(int argc, char** argv)
{
  // Parse arguments
  if (argc != 2)
    return 1;
  const std::string input_fname = argv[1];

  // Read input file data into memory
  utils::byte_vec input_data;
  if (!utils::read_file(input_fname, input_data))
    return 1;

  // Scan through input data and parse as transport stream format
  mpegts::StreamMap_t streams;
  if (!mpegts::demux(utils::byte_vec_to_view(input_data), streams))
    return 1;
  input_data.clear();
  for (const auto& [pid, stream] : streams)
    std::cout << "found stream pid=" << pid << " num_packets=" << stream.Packets.size() << std::endl;

  // Ignore the SDT (service description table)
  streams.erase(mpegts::PID_SDT);

  // Scan the PAT (program association table) to find the PMT PID
  uint16_t pmt_pid = 0;
  if (streams.count(mpegts::PID_PAT) == 0) {
    std::cerr << "transport stream contained no PAT" << std::endl;
    return 1;
  }
  if (!mpegts::parse_pat(streams[mpegts::PID_PAT], pmt_pid))
    return 1;
  streams.erase(mpegts::PID_PAT);
  std::cout << "finished scanning PATs, found PMT pid=" << pmt_pid << std::endl;

  // Scan the PMT (program map table) to tag all the streams
  if (streams.count(pmt_pid) == 0) {
    std::cerr << "transport stream contained no PMT" << std::endl;
    return 1;
  }
  if (!mpegts::parse_pmt(streams[pmt_pid], streams))
    return 1;
  streams.erase(pmt_pid);

  // Print stream type debugging information and find singular MPEG2 video stream for cc extraction
  static constexpr uint8_t STREAM_TYPE_VIDEO_MPEG2 = 2;
  static constexpr uint8_t STREAM_TYPE_AUDIO_AC3   = 129;
  for (const auto& [pid, stream] : streams) {
    if (stream.Type == 0) {
      std::cerr << "unidentified stream pid=" << pid << std::endl;
      return 1;
    } else if (stream.Type == STREAM_TYPE_VIDEO_MPEG2) {
      std::cout << "identified stream pid=" << pid << " type=Video_MPEG2" << std::endl;
    } else if (stream.Type == STREAM_TYPE_AUDIO_AC3) {
      std::cout << "identified stream pid=" << pid << " type=Audio_AC3 lang=(" << stream.Lang << ")" << std::endl;
    } else {
      std::cerr << "unknown stream type pid=" << pid << " type=" << int(stream.Type) << std::endl;
      return 1;
    }
  }
  for (auto it = streams.begin(); it != streams.end(); ) {
    if (it->second.Type != STREAM_TYPE_VIDEO_MPEG2)
      it = streams.erase(it);
    else
      ++it;
  }
  if (streams.size() != 1) {
    std::cerr << "did not find singular MPEG2 video stream" << std::endl;
    return 1;
  }

  // Extract the elementary stream into a contiguous region of memory
  utils::StreamBuffer mpeg2_stream;
  if (!depacketize_stream(streams.cbegin()->second.Packets, mpeg2_stream))
    return 1;
  streams.clear();
  std::cout << "depacketized mpeg2 stream bytes=" << mpeg2_stream.getReadLeft() << std::endl;

  // Extract DTVCC transport stream (CEA-708 closed captions) from MPEG2 video stream
  std::vector<utils::StreamBuffer> dtvcc_packets;
  if (!extract_dtvcc_packets(mpeg2_stream, dtvcc_packets))
    return 1;
  mpeg2_stream.clear();
  std::cout << "extracted dtvcc transport stream num_packets=" << dtvcc_packets.size() << std::endl;

  return 0;
}

static bool depacketize_stream(const std::vector<utils::StreamBuffer>& packets, utils::StreamBuffer& out)
{
  for (const auto& pkt : packets) {
    std::basic_string_view<uint8_t> pkt_iter((const uint8_t*) pkt.getRead(), pkt.getReadLeft());
    if (pkt_iter.size() < 9) {
      std::cerr << "invalid pes header" << std::endl;
      return false;
    }
    const uint32_t start_code =
      (pkt_iter[0] << 16) |
      (pkt_iter[1] <<  8) |
      (pkt_iter[2] <<  0);
    const uint8_t stream_id = pkt_iter[3];
    const uint16_t pkt_len =
      (pkt_iter[4] <<  8) |
      (pkt_iter[5] <<  0);
    const uint8_t marker              = (pkt_iter[6] >> 6) & 0b11;
    const uint8_t scrambling_control  = (pkt_iter[6] >> 4) & 0b11;
    const bool priority               = pkt_iter[6] & 0x08;
    const bool alignment_ind          = pkt_iter[6] & 0x04;
    const bool copyright              = pkt_iter[6] & 0x02;
    const bool original_copy          = pkt_iter[6] & 0x01;
    const uint8_t pts_dts             = (pkt_iter[7] >> 6) & 0b11;
    const bool escr_flag              = pkt_iter[7] & 0x20;
    const bool es_rate                = pkt_iter[7] & 0x10;
    const bool dsm_trick_mode         = pkt_iter[7] & 0x08;
    const bool additional_copy        = pkt_iter[7] & 0x04;
    const bool crc_flag               = pkt_iter[7] & 0x02;
    const bool ext_flag               = pkt_iter[7] & 0x01;
    const uint8_t pes_hdr_len         = pkt_iter[8];
    if (start_code != 0x000001 ||
        stream_id != 0xE0 ||
        pkt_len != 0 ||
        marker != 0b10 ||
        scrambling_control != 0b00 ||
        priority ||
        alignment_ind ||
        copyright ||
        original_copy ||
        (pts_dts != 0b11 && pts_dts != 0b10) ||
        escr_flag ||
        es_rate ||
        dsm_trick_mode ||
        additional_copy ||
        crc_flag ||
        ext_flag ||
        (pts_dts == 0b11 && pes_hdr_len != 10) ||
        (pts_dts == 0b10 && pes_hdr_len != 5) ||
        pkt_iter.size() < 9 + pes_hdr_len) {
      std::cerr << "invalid pes header" << std::endl;
      return false;
    }
    //TODO: Parse & store PTS/DTS
    pkt_iter.remove_prefix(9 + pes_hdr_len);
    out.write(pkt_iter.data(), pkt_iter.size());
  }
  return true;
}

static bool extract_dtvcc_packets(const utils::StreamBuffer& mpeg2_stream, std::vector<utils::StreamBuffer>& out)
{
  std::basic_string_view<uint8_t> iter((const uint8_t*) mpeg2_stream.getRead(), mpeg2_stream.getReadLeft());

  while (iter.size() >= 4)
  {
    // MPEG2 Video Data Layers
    // -----------------------
    // [] Sequence
    //   Sequence Header
    //   Sequence Extension Header(s)
    //   [] Group of Pictures
    //     Group of Pictures Header
    //     [] Picture
    //       Picture Header
    //       Picture Coding Extension Header
    //       [] User
    //         User header
    //         user_data header
    //         [] cc_data_pkt
    //         user_data footer
    //       [] Slice
    //         Slice Header
    //         [] Macroblock
    // Sequence End Code

    if (!parse_mpeg2_sequence(iter, out))
      return false;
  }

  if (!out.empty() && out.back().getReadLeft() <= 0)
    out.pop_back();

  if (out.empty()) {
    std::cerr << "no dtvcc packets found" << std::endl;
    return false;
  }

  return true;
}

static bool parse_mpeg2_sequence(std::basic_string_view<uint8_t>& iter, std::vector<utils::StreamBuffer>& out)
{
  // Sequence header
  {
    if (iter.size() < 12) {
      std::cerr << "invalid sequence header" << std::endl;
      return false;
    }
    const uint32_t start_code =
      (iter[0] << 24) |
      (iter[1] << 16) |
      (iter[2] << 8)  |
      (iter[3] << 0);
    const uint16_t horz_size        = (iter[4] << 4) | (iter[5] >> 4);
    const uint16_t vert_size        = ((iter[5] & 0x0F) << 8) | iter[6];
    const uint8_t aspect_ratio      = (iter[7] >> 4) & 0x0F;
    const uint8_t frame_rate        = iter[7] & 0x0F;
    const uint32_t bit_rate =
      (iter[8] << 10) |
      (iter[9] << 2) |
      ((iter[10] >> 6) & 0b11);
    const bool reserved0            = iter[10] & 0x20;
    const uint16_t vbv_buf_size =
      ((iter[10] & 0x1F) << 5) |
      ((iter[11] >> 3) & 0b11111);
    const bool constrained_params   = iter[11] & 0x04;
    if (start_code != START_CODE_SEQ ||
        //horz_size != 1920 || TODO
        //vert_size != 1080 || TODO
        aspect_ratio != 0x3 || // 16:9
        //frame_rate != 0x4 || // 29.9 TODO
        //bit_rate TODO
        !reserved0 ||
        //vbv_buf_size TODO
        constrained_params) {
      std::cerr << "invalid sequence header" << std::endl;
      return false;
    }

    size_t req_bytes = 12;
    const bool intra_quantiser = iter[11] & 0x02;
    if (intra_quantiser)
      req_bytes += 64;
    if (iter.size() < req_bytes) {
      std::cerr << "invalid sequence header" << std::endl;
      return false;
    }
    const bool non_intra_quantiser = iter[req_bytes - 1] & 0x01;
    if (non_intra_quantiser)
      req_bytes += 64;
    if (iter.size() < req_bytes) {
      std::cerr << "invalid sequence header" << std::endl;
      return false;
    }
    iter.remove_prefix(req_bytes);
  }

  // Sequence extension header
  {
    if (iter.size() < 10) {
      std::cerr << "invalid sequence extension header" << std::endl;
      return false;
    }
    const uint32_t start_code =
      (iter[0] << 24) |
      (iter[1] << 16) |
      (iter[2] << 8)  |
      (iter[3] << 0);
    const uint8_t variety = (iter[4] >> 4) & 0x0F;
    //TODO: Other fields
    if (start_code != START_CODE_EXT ||
        variety != 0b0001) {
      std::cerr << "invalid sequence extension header" << std::endl;
      return false;
    }
    iter.remove_prefix(10);
  }

  // Array of group of pictures structures
  size_t count = 0;
  while (iter.size() >= 4) {
    const uint32_t start_code =
      (iter[0] << 24) |
      (iter[1] << 16) |
      (iter[2] << 8)  |
      (iter[3] << 0);
    if (start_code == START_CODE_GOP) {
      if (!parse_mpeg2_gop(iter, out))
        return false;
      ++count;
    }
    else if (start_code == START_CODE_SEQ) {
      break;
    }
    else {
      std::cerr << "invalid start code in sequence structure" << std::endl;
      return false;
    }
  }

  if (count == 0) {
    std::cerr << "invalid sequence, no gop structures found" << std::endl;
    return false;
  }

  return true;
}

static bool parse_mpeg2_gop(std::basic_string_view<uint8_t>& iter, std::vector<utils::StreamBuffer>& out)
{
  // Group of pictures header
  {
    if (iter.size() < 8) {
      std::cerr << "invalid gop header" << std::endl;
      return false;
    }
    const uint32_t start_code =
      (iter[0] << 24) |
      (iter[1] << 16) |
      (iter[2] << 8)  |
      (iter[3] << 0);
    //TODO: Other fields
    if (start_code != START_CODE_GOP) {
      std::cerr << "invalid gop header" << std::endl;
      return false;
    }
    iter.remove_prefix(8);
  }

  // Array of picture structures
  size_t count = 0;
  while (iter.size() >= 4) {
    const uint32_t start_code =
      (iter[0] << 24) |
      (iter[1] << 16) |
      (iter[2] << 8)  |
      (iter[3] << 0);
    if (start_code == START_CODE_PIC) {
      if (!parse_mpeg2_picture(iter, out))
        return false;
      ++count;
    }
    else if (start_code == START_CODE_GOP ||
             start_code == START_CODE_SEQ) {
      break;
    }
    else {
      std::cerr << "invalid start code in gop structure" << std::endl;
      return false;
    }
  }

  if (count == 0) {
    std::cerr << "invalid gop, no picture structures found" << std::endl;
    return false;
  }

  return true;
}

static bool parse_mpeg2_picture(std::basic_string_view<uint8_t>& iter, std::vector<utils::StreamBuffer>& out)
{
  // Picture header
  {
    if (iter.size() < 8) {
      std::cerr << "invalid picture header" << std::endl;
      return false;
    }
    const uint32_t start_code =
      (iter[0] << 24) |
      (iter[1] << 16) |
      (iter[2] << 8)  |
      (iter[3] << 0);
    //TODO: Other fields
    if (start_code != START_CODE_PIC) {
      std::cerr << "invalid picture header" << std::endl;
      return false;
    }
    iter.remove_prefix(8);
    if (iter[0] != 0) //TODO
      iter.remove_prefix(1);
  }

  // Picture coding extension header
  {
    if (iter.size() < 11) {
      std::cerr << "invalid picture coding extension header" << std::endl;
      return false;
    }
    const uint32_t start_code =
      (iter[0] << 24) |
      (iter[1] << 16) |
      (iter[2] << 8)  |
      (iter[3] << 0);
    const uint8_t variety = (iter[4] >> 4) & 0x0F;
    //TODO: Other fields and checks
    if (start_code != START_CODE_EXT ||
        variety != 0b1000) {
      std::cerr << "invalid picture coding extension header" << std::endl;
      return false;
    }
    const bool composite_display = iter[8] & 0x40;
    iter.remove_prefix(composite_display ? 11 : 9);
  }

  // Array of user / slice structures
  size_t count = 0;
  while (iter.size() >= 4) {
    const uint32_t start_code =
      (iter[0] << 24) |
      (iter[1] << 16) |
      (iter[2] << 8)  |
      (iter[3] << 0);
    if (start_code == START_CODE_USR) {
      if (!parse_mpeg2_user(iter, out))
        return false;
      ++count;
    }
    else if (start_code >= START_CODE_SLL && start_code <= START_CODE_SLH) {
      if (!parse_mpeg2_slice(iter))
        return false;
      ++count;
    }
    else if (start_code == START_CODE_PIC ||
             start_code == START_CODE_GOP ||
             start_code == START_CODE_SEQ) {
      break;
    }
    else {
      std::cerr << "invalid start code in picture structure" << std::endl;
      return false;
    }
  }

  if (count == 0) {
    std::cerr << "invalid picture, no user structures nor slices found" << std::endl;
    return false;
  }

  return true;
}

static bool parse_mpeg2_slice(std::basic_string_view<uint8_t>& iter)
{
  //TODO: Just skipping over macroblock stuff for now
  iter.remove_prefix(4);
  while (iter.size() >= 3) {
    const uint32_t start_code =
      (iter[0] << 16) |
      (iter[1] << 8)  |
      (iter[2] << 0);
    if (start_code == 0x000001)
      return true;
    iter.remove_prefix(1);
  }
  return true;
}

static bool parse_mpeg2_user(std::basic_string_view<uint8_t>& iter, std::vector<utils::StreamBuffer>& out)
{
  // User header
  {
    if (iter.size() < 9) {
      std::cerr << "invalid user header" << std::endl;
      return false;
    }
    const uint32_t start_code =
      (iter[0] << 24) |
      (iter[1] << 16) |
      (iter[2] << 8)  |
      (iter[3] << 0);
    const uint32_t atsc_ident =
      (iter[4] << 24) |
      (iter[5] << 16) |
      (iter[6] << 8)  |
      (iter[7] << 0);
    if (start_code != START_CODE_USR) {
      std::cerr << "invalid user header" << std::endl;
      return false;
    }

    //TODO
    if (atsc_ident == *((uint32_t*)"1GTD")) {
      iter.remove_prefix(8);
      while (iter.size() >= 3) {
        const uint32_t start_code =
          (iter[0] << 16) |
          (iter[1] << 8)  |
          (iter[2] << 0);
        if (start_code == 0x000001)
          break;
        iter.remove_prefix(1);
      }
      return true;
    }

    const uint8_t type_code = iter[8];
    if (atsc_ident != *((uint32_t*)"49AG") || // Backwards to flip endianness
        type_code != 3) {
      std::cerr << "invalid user atsc a53 header" << std::endl;
      return false;
    }
    iter.remove_prefix(9);
  }

  // user_data header
  uint8_t cc_count = 0;
  {
    if (iter.size() < 3) {
      std::cerr << "invalid user_data header" << std::endl;
      return false;
    }
    const bool process_em_data = iter[0] & 0x80;
    const bool process_cc_data = iter[0] & 0x40;
    const bool process_addl_data = iter[0] & 0x20;
    cc_count = iter[0] & 0b11111;
    const uint8_t em_data = iter[1];
    if (!process_em_data ||
        !process_cc_data ||
        process_addl_data ||
        cc_count == 0 ||
        em_data != 0xFF) {
      std::cerr << "invalid user_data header" << std::endl;
      return false;
    }
    iter.remove_prefix(2);
  }

  // Array of cc_data_pkt structures
  for (unsigned i = 0; i < cc_count; ++i)
  {
    if (iter.size() < 3) {
      std::cerr << "invalid cc_data_pkt" << std::endl;
      return false;
    }
    const uint8_t marker = (iter[0] >> 3) & 0b11111;
    const bool cc_valid = iter[0] & 0x04;
    const uint8_t cc_type = iter[0] & 0b11;
    const uint8_t data1 = iter[1];
    const uint8_t data2 = iter[2];
    if (marker != 0b11111) {
      std::cerr << "invalid cc_data_pkt" << std::endl;
      return false;
    }
    iter.remove_prefix(3);

    if (cc_type == 0 || cc_type == 1)
      continue; // Backwards-compatible CEA-608 / NTSC

    // Start a new DTVCC packet when instructed to do so
    if (!cc_valid || cc_type == 3)
      if (out.empty() || out.back().getReadLeft() > 0)
        out.emplace_back(utils::StreamBuffer());

    if (cc_valid) {
      if (out.empty()) {
        std::cerr << "invalid cc_data_pkt" << std::endl;
        return false;
      }
      out.back().write(&data1, 1);
      out.back().write(&data2, 1);
    }
  }

  // user_data footer
  {
    if (iter.size() < 1) {
      std::cerr << "invalid user_data footer" << std::endl;
      return false;
    }
    const uint8_t marker_bits = iter[0];
    if (marker_bits != 0xFF) {
      std::cerr << "invalid user_data footer" << std::endl;
      return false;
    }
    iter.remove_prefix(1);
  }

  return true;
}
