////////////////////////////////////////////////////////////////////////////////
/// atsc-cc-extractor
///
/// Program that outputs embedded CEA-708 closed captions in SRT format from
/// over-the-air ATSC 1.0 TV recordings captured in MPEG-TS format.

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

int main(int argc, char** argv)
{
  // Parse arguments
  if (argc < 2)
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

  // Print stream debugging information
  for (const auto& [pid, stream] : streams)
    std::cout << "found stream pid=" << pid << " packets=" << stream.Packets.size() << std::endl;

  // Ignore SDT (service description table)
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

  // Scan the PMT (program map table)
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
    } else if (stream.Type == STREAM_TYPE_VIDEO_MPEG2) {
      std::cout << "identified stream pid=" << pid << " type=Video_MPEG2" << std::endl;
    } else if (stream.Type == STREAM_TYPE_AUDIO_AC3) {
      std::cout << "identified stream pid=" << pid << " type=Audio_AC3 lang=(" << stream.Lang << ")" << std::endl;
    } else {
      std::cerr << "unknown stream type pid=" << pid << " type=" << int(stream.Type) << std::endl;
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

  //TODO: Scan the MPEG2 video stream for embedded CEA-708 closed captions

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
          //TODO: std::cout << "pcr field stream=" << pid << " pcr=" << PCR << std::endl;
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
