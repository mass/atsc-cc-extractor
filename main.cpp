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

#include <cstring>
#include <iostream>
#include <memory>
#include <unordered_map>
#include <vector>

static constexpr uint16_t PID_NULL_PKT = 0x1FFF;
static constexpr uint16_t PID_PAT      = 0x0000;
static constexpr uint16_t PID_SDT      = 0x0011;

static constexpr uint8_t STREAM_TYPE_VIDEO_MPEG2 = 2;
static constexpr uint8_t STREAM_TYPE_AUDIO_AC3   = 129;

static constexpr uint8_t TABLE_ID_PAT = 0;
static constexpr uint8_t TABLE_ID_PMT = 2;

static constexpr uint32_t START_CODE_SEQ = 0x000001B3;
static constexpr uint32_t START_CODE_EXT = 0x000001B5;
static constexpr uint32_t START_CODE_GOP = 0x000001B8;
static constexpr uint32_t START_CODE_PIC = 0x00000100;
static constexpr uint32_t START_CODE_USR = 0x000001B2;
static constexpr uint32_t START_CODE_SLL = 0x00000101;
static constexpr uint32_t START_CODE_SLH = 0x000001AF;

// Elementary stream (video/audio/metadata/etc.)
struct Stream
{
  uint8_t SeqNum = 0xF;
  uint8_t Type = 0;
  std::string Lang;
  std::vector<StreamBuffer> Packets;
};

static bool read_file(std::string_view fname, std::vector<uint8_t>& out);
static bool demux_mpegts(const std::vector<uint8_t>& data, std::unordered_map<uint16_t, Stream>& out);
static bool parse_table(const StreamBuffer& pkt, uint8_t expected_table_id, std::basic_string_view<uint8_t>& table_data);
static bool parse_pat(const Stream& pat, uint16_t& pmt_pid);
static bool parse_pmt(const Stream& pmt, std::unordered_map<uint16_t, Stream>& streams);
static bool depacketize_stream(const std::vector<StreamBuffer>& packets, StreamBuffer& out);
static bool extract_dtvcc_packets(const StreamBuffer& mpeg2_stream, std::vector<StreamBuffer>& out);
static bool parse_mpeg2_sequence(std::basic_string_view<uint8_t>& iter, std::vector<StreamBuffer>& out);
static bool parse_mpeg2_gop(std::basic_string_view<uint8_t>& iter, std::vector<StreamBuffer>& out);
static bool parse_mpeg2_picture(std::basic_string_view<uint8_t>& iter, std::vector<StreamBuffer>& out);
static bool parse_mpeg2_slice(std::basic_string_view<uint8_t>& iter);
static bool parse_mpeg2_user(std::basic_string_view<uint8_t>& iter, std::vector<StreamBuffer>& out);

int main(int argc, char** argv)
{
  // Parse arguments
  if (argc != 2)
    return 1;
  const std::string input_fname = argv[1];

  // Read input file data into memory
  std::vector<uint8_t> input_data;
  if (!read_file(input_fname, input_data))
    return 1;

  // Scan through input data and parse as transport stream format
  std::unordered_map<uint16_t, Stream> streams;
  if (!demux_mpegts(input_data, streams))
    return 1;
  input_data.clear();
  for (const auto& [pid, stream] : streams)
    std::cout << "found stream pid=" << pid << " num_packets=" << stream.Packets.size() << std::endl;

  // Ignore the SDT (service description table)
  streams.erase(PID_SDT);

  // Scan the PAT (program association table) to find the PMT PID
  uint16_t pmt_pid = 0;
  if (streams.count(PID_PAT) == 0) {
    std::cerr << "transport stream contained no PAT" << std::endl;
    return 1;
  }
  if (!parse_pat(streams[PID_PAT], pmt_pid))
    return 1;
  streams.erase(PID_PAT);
  std::cout << "finished scanning PATs, found PMT pid=" << pmt_pid << std::endl;

  // Scan the PMT (program map table) to tag all the streams
  if (streams.count(pmt_pid) == 0) {
    std::cerr << "transport stream contained no PMT" << std::endl;
    return 1;
  }
  if (!parse_pmt(streams[pmt_pid], streams))
    return 1;
  streams.erase(pmt_pid);

  // Print stream type debugging information and find singular MPEG2 video stream for cc extraction
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
  StreamBuffer mpeg2_stream;
  if (!depacketize_stream(streams.cbegin()->second.Packets, mpeg2_stream))
    return 1;
  streams.clear();
  std::cout << "depacketized mpeg2 stream bytes=" << mpeg2_stream.getReadLeft() << std::endl;

  // Extract DTVCC transport stream (CEA-708 closed captions) from MPEG2 video stream
  std::vector<StreamBuffer> dtvcc_packets;
  if (!extract_dtvcc_packets(mpeg2_stream, dtvcc_packets))
    return 1;
  mpeg2_stream.clear();
  std::cout << "extracted dtvcc transport stream num_packets=" << dtvcc_packets.size() << std::endl;

  return 0;
}

static bool read_file(std::string_view fname, std::vector<uint8_t>& out)
{
  FILE* fp = nullptr;
  if ((fp = ::fopen(fname.data(), "r")) == nullptr) {
    std::cerr << "error opening file error=(" << std::strerror(errno) << ")" << std::endl;
    return false;
  }

  size_t file_size = 0;
  if (::fseek(fp, 0, SEEK_END) != 0 || (file_size = ::ftell(fp)) < 0 || ::fseek(fp, 0, SEEK_SET) != 0) {
    std::cerr << "error finding file size error=(" << std::strerror(errno) << ")" << std::endl;
    return false;
  }

  out.resize(file_size);
  if (::fread(out.data(), 1, file_size, fp) != file_size) {
    std::cerr << "error reading file error=(" << std::strerror(errno) << ")" << std::endl;
    out.clear();
    return false;
  }

  ::fclose(fp);

  return true;
}

static bool demux_mpegts(const std::vector<uint8_t>& data, std::unordered_map<uint16_t, Stream>& out)
{
  static constexpr size_t TS_PKT_SIZE = 188;

  std::basic_string_view<uint8_t> iter(data.data(), data.size());

  while (iter.size() > 0) {
    if (iter.size() < TS_PKT_SIZE) {
      std::cerr << "invalid input data bytes_left=" << iter.size() << std::endl;
      return false;
    }
    std::basic_string_view<uint8_t> pkt_iter(iter.data(), TS_PKT_SIZE);
    iter.remove_prefix(TS_PKT_SIZE);

    // Parse & verify header fields
    const uint8_t sync_byte = pkt_iter[0];
    const bool transport_error = pkt_iter[1] & 0x80;
    const bool payload_start = pkt_iter[1] & 0x40;
    const bool transport_pri = pkt_iter[1] & 0x20;
    const uint16_t pid = ((pkt_iter[1] & 0x1F) << 8) | pkt_iter[2];
    const uint8_t transport_scrambling = (pkt_iter[3] >> 6) & 0b11;
    const uint8_t adaptation_field = (pkt_iter[3] >> 4) & 0b11;
    const uint8_t continuity_counter = pkt_iter[3] & 0b1111;
    if (sync_byte != 'G' ||
        transport_error ||
        transport_pri ||
        transport_scrambling != 0b00 ||
        adaptation_field == 0b00) {
      std::cerr << "invalid ts packet" << std::endl;
      return false;
    }
    pkt_iter.remove_prefix(4);

    auto i_stream = out.find(pid);
    if (i_stream == out.end())
      i_stream = out.insert({pid, Stream()}).first;
    auto& stream = i_stream->second;

    // Parse adaptation field if it exists
    if (adaptation_field == 0b10 || adaptation_field == 0b11) {
      const uint8_t adaptation_len = pkt_iter[0];
      pkt_iter.remove_prefix(1);
      if ((adaptation_field == 0b10 && adaptation_len == 0) ||
          (adaptation_field == 0b11 && adaptation_len >= pkt_iter.size()) ||
          (adaptation_len > pkt_iter.size())) {
        std::cerr << "invalid adaptation field length" << std::endl;
        return false;
      }
      if (adaptation_len > 0) {
        const bool discontinuity         = pkt_iter[0] & 0x80;
        const bool random_access         = pkt_iter[0] & 0x40;
        const bool stream_pri            = pkt_iter[0] & 0x20;
        const bool pcr_flag              = pkt_iter[0] & 0x10;
        const bool opcr_flag             = pkt_iter[0] & 0x08;
        const bool splicing_point        = pkt_iter[0] & 0x04;
        const bool transport_private     = pkt_iter[0] & 0x02;
        const bool adaptation_extension  = pkt_iter[0] & 0x01;
        if (discontinuity ||
            stream_pri ||
            opcr_flag ||
            splicing_point ||
            transport_private ||
            adaptation_extension) {
          std::cerr << "invalid adaptation field flags" << std::endl;
          return false;
        }

        if (pcr_flag) {
          if (adaptation_len != 7) {
            std::cerr << "invalid adaptation field pcr" << std::endl;
            return false;
          }
          //const uint64_t PCR = 0; //TODO: 33 bits base, 6 rsvd, 9 ext ; PCR = base * 300 + ext
        }
        else if (random_access) {
          if (adaptation_len != 1) {
            std::cerr << "invalid adaptation field random_access" << std::endl;
            return false;
          }
          //TODO: Do anything w/ this field?
        }
        else {
          for (uint8_t i = 1; i < adaptation_len; ++i) {
            if (pkt_iter[i] != 0xFF) {
              std::cerr << "invalid adaptation field stuffing bytes" << std::endl;
              return false;
            }
          }
        }

        pkt_iter.remove_prefix(adaptation_len);
      }
    }

    // Ignore null packets inserted for constant bit rates
    if (pid == PID_NULL_PKT)
      continue;

    // Verify sequence number continuity per stream
    if (continuity_counter != ((stream.SeqNum + 1) & 0xF)) {
      std::cerr << "sequence error for stream pid=" << pid
                << " expected=" << int((stream.SeqNum + 1) & 0xF)
                << " received=" << int(continuity_counter) << std::endl;
      return false;
    }
    stream.SeqNum = continuity_counter;

    // Parse packet payload
    if (!(adaptation_field & 0b1)) {
      std::cerr << "invalid packet, no payload";
      return false;
    }
    else {
      if (payload_start)
        stream.Packets.emplace_back(StreamBuffer());
      else if (stream.Packets.empty()) {
        std::cerr << "payload not started with no previous packet stream=" << pid << std::endl;
        return false;
      }
      auto& stream_packet = stream.Packets.back();
      stream_packet.write(pkt_iter.data(), pkt_iter.size());
    }
  }

  return true;
}

static bool parse_table(const StreamBuffer& pkt, uint8_t expected_table_id, std::basic_string_view<uint8_t>& table_data)
{
  std::basic_string_view<uint8_t> pkt_iter((const uint8_t*) pkt.getRead(), pkt.getReadLeft());
  if (pkt_iter.empty()) {
    std::cerr << "invalid table" << std::endl;
    return false;
  }

  const uint8_t pointer_bytes = pkt_iter[0];
  if (pointer_bytes != 0) {
    std::cerr << "unhandled table pointer bytes num=" << int(pointer_bytes) << std::endl;
    return false;
  }
  pkt_iter.remove_prefix(1);

  if (pkt_iter.size() < 3) {
    std::cerr << "invalid table" << std::endl;
    return false;
  }
  const uint8_t table_id = pkt_iter[0];
  const bool section_syntax_flag = pkt_iter[1] & 0x80;
  //const bool private_flag        = pkt_iter[1] & 0x40;
  const uint8_t reserved0        = (pkt_iter[1] >> 2) & 0b1111;
  const uint16_t section_len = ((pkt_iter[1] & 0b11) << 8) | pkt_iter[2];
  if (table_id != expected_table_id ||
      ! section_syntax_flag ||
      reserved0 != 0b1100 ||
      section_len > 1021 ||
      section_len < 9 ||
      section_len > pkt_iter.size()) {
    std::cerr << "invalid table fields" << std::endl;
    return false;
  }
  pkt_iter.remove_prefix(3);

  const uint16_t data_len = section_len - 9;
  const uint16_t table_id_ext = (pkt_iter[0] << 8) | pkt_iter[1];
  const uint8_t reserved1 = (pkt_iter[2] >> 6) & 0b11;
  const uint8_t version = (pkt_iter[2] >> 1) & 0b11111;
  const bool current_next = pkt_iter[2] & 0b1;
  const uint8_t section_num = pkt_iter[3];
  const uint8_t last_section_num = pkt_iter[4];
  table_data = std::basic_string_view<uint8_t>(pkt_iter.data() + 5, data_len);
  const uint32_t crc = //TODO: Endianness, Check
    (pkt_iter[5 + data_len + 0] << 24) |
    (pkt_iter[5 + data_len + 1] << 16) |
    (pkt_iter[5 + data_len + 2] << 8)  |
    (pkt_iter[5 + data_len + 3] << 0);
  if (table_id_ext != 1 ||
      reserved1 != 0b11 ||
      version != 0 ||
      ! current_next ||
      section_num != 0 ||
      last_section_num != 0) {
    std::cerr << "invalid section fields" << std::endl;
    return false;
  }
  pkt_iter.remove_prefix(section_len);

  // All remaining bytes should be stuffed
  while (!pkt_iter.empty()) {
    if (pkt_iter[0] != 0xFF) {
      std::cerr << "invalid table stuffing" << std::endl;
      return false;
    }
    pkt_iter.remove_prefix(1);
  }

  return true;
}

static bool parse_pat(const Stream& pat, uint16_t& pmt_pid)
{
  for (const auto& pkt : pat.Packets) {
    std::basic_string_view<uint8_t> table_data;
    if (!parse_table(pkt, TABLE_ID_PAT, table_data))
      return false;

    const uint16_t program_num = (table_data[0] << 8) | table_data[1];
    const uint8_t reserved2 = (table_data[2] >> 5) & 0b111;
    const uint16_t pid = ((table_data[2] & 0b11111) << 8) | table_data[3];
    if (program_num != 1 ||
        reserved2 != 0b111 ||
        (pmt_pid != 0 && pmt_pid != pid)) {
      std::cerr << "invalid PAT table" << std::endl;
      return false;
    }
    pmt_pid = pid;
  }
  return true;
}

static bool parse_pmt(const Stream& pmt, std::unordered_map<uint16_t, Stream>& streams)
{
  for (const auto& pkt : pmt.Packets) {
    std::basic_string_view<uint8_t> table_data;
    if (!parse_table(pkt, TABLE_ID_PMT, table_data))
      return false;

    if (table_data.size() < 4) {
      std::cerr << "invalid PMT table" << std::endl;
      return false;
    }
    const uint8_t reserved2 = (table_data[0] >> 5) & 0b111;
    //const uint16_t pcr_pid = ((table_data[0] & 0b11111) << 8) | table_data[1];
    const uint8_t reserved3 = (table_data[2] >> 2) & 0b111111;
    const uint16_t pid_len = ((table_data[2] & 0b11) << 8) | table_data[3];
    if (reserved2 != 0b111 ||
        reserved3 != 0b111100 ||
        pid_len != 0) {
      std::cerr << "invalid PMT table header" << std::endl;
      return false;
    }
    table_data.remove_prefix(4);

    while (!table_data.empty()) {
      if (table_data.size() < 5) {
        std::cerr << "invalid PMT table stream data" << std::endl;
        return false;
      }
      const uint8_t stream_type = table_data[0];
      const uint8_t reserved4 = (table_data[1] >> 5) & 0b111;
      const uint16_t stream_pid = ((table_data[1] & 0b11111) << 8) | table_data[2];
      const uint8_t reserved5 = (table_data[3] >> 2) & 0b111111;
      const uint16_t stream_info_len = ((table_data[3] & 0b11) << 8) | table_data[4];
      if (reserved4 != 0b111 ||
          reserved5 != 0b111100 ||
          stream_info_len + 5 > table_data.size()) {
        std::cerr << "invalid PMT table stream fields" << std::endl;
        return false;
      }
      std::basic_string_view<uint8_t> stream_info(table_data.data() + 5, stream_info_len);
      table_data.remove_prefix(5 + stream_info_len);

      auto i_stream = streams.find(stream_pid);
      if (i_stream == streams.end()) {
        std::cerr << "unknown stream in PMT pid=" << int(stream_pid) << std::endl;
        return false;
      }
      auto& stream = i_stream->second;
      if (stream.Type != 0 && stream.Type != stream_type) {
        std::cerr << "conflicting stream types from PMT pid=" << int(stream_pid) << std::endl;
        return false;
      }
      stream.Type = stream_type;

      static constexpr uint8_t DESC_TAG_LANG = 10;

      // Read stream_info descriptors
      while (!stream_info.empty()) {
        if (stream_info.size() < 2) {
          std::cerr << "invalid PMT stream info descriptor" << std::endl;
          return false;
        }
        const uint8_t desc_tag = stream_info[0];
        const uint8_t desc_len = stream_info[1];
        stream_info.remove_prefix(2);

        if (desc_len > stream_info.size()) {
          std::cerr << "invalid PMT stream info descriptor" << std::endl;
          return false;
        }

        if (desc_tag == DESC_TAG_LANG) {
          if (stream_info[desc_len-1] != 0x00 && stream_info[desc_len-1] != 0x03) {
            std::cerr << "invalid PMT stream info lang descriptor" << std::endl;
            return false;
          }
          std::string_view lang((const char*) stream_info.data(), desc_len - 1);
          if (!stream.Lang.empty() && stream.Lang != lang) {
            std::cerr << "conflicting stream info lang descriptor pid=" << int(stream_pid) << std::endl;
            return false;
          }
          stream.Lang = lang;
        } else {
          std::cerr << "unhandled stream descriptor tag=" << int(desc_tag) << std::endl;
          return false;
        }
        stream_info.remove_prefix(desc_len);
      }
    }
  }
  return true;
}

static bool depacketize_stream(const std::vector<StreamBuffer>& packets, StreamBuffer& out)
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

static bool extract_dtvcc_packets(const StreamBuffer& mpeg2_stream, std::vector<StreamBuffer>& out)
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

static bool parse_mpeg2_sequence(std::basic_string_view<uint8_t>& iter, std::vector<StreamBuffer>& out)
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

static bool parse_mpeg2_gop(std::basic_string_view<uint8_t>& iter, std::vector<StreamBuffer>& out)
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

static bool parse_mpeg2_picture(std::basic_string_view<uint8_t>& iter, std::vector<StreamBuffer>& out)
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

static bool parse_mpeg2_user(std::basic_string_view<uint8_t>& iter, std::vector<StreamBuffer>& out)
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
        out.emplace_back(StreamBuffer());

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
