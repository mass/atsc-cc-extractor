#pragma once

#include <m/log.hpp>
#include <m/stream_buf.hpp>

#include <map>
#include <unordered_map>

////////////////////////////////////////////////////////////////////////////////
/// Utilities for working with data in the MPEG-TS digital container format.
namespace mpegts
{
  // Packet Identifier - uniquely identifies a table or elementary stream
  using Pid_t = uint16_t;
  static constexpr Pid_t PID_PAT   = 0x0000;
  static constexpr Pid_t PID_CAT   = 0x0001;
  static constexpr Pid_t PID_TSDT  = 0x0002;
  static constexpr Pid_t PID_IPMP  = 0x0003;
  static constexpr Pid_t PID_SDT   = 0x0011;
  static constexpr Pid_t PID_NULL  = 0x1FFF;

  // Packetized Elementary Stream - carries a single video/audio/data stream
  struct PacketizedStream
  {
    uint8_t SeqNum = 0xF;
    uint8_t Type = 0;
    std::string Lang;

    struct Packet
    {
      uint64_t PCR;
      m::stream_buf<> Data;
    };
    std::vector<Packet> Packets;
  };
  using StreamMap_t = std::unordered_map<Pid_t, PacketizedStream>;

  // Elementary Stream - entire stream in one contiguous region of memory
  struct ElementaryStream
  {
    m::stream_buf<> Data;

    struct Timecodes
    {
      uint64_t PCR = UINT64_MAX;
      uint64_t PTS = UINT64_MAX;
      uint64_t DTS = UINT64_MAX;
    };
    std::map<off_t, Timecodes> Times;
  };

  // Demultiplex bytes in MPEG-TS container format into a PES map
  bool demux(m::byte_view input, StreamMap_t& out, Pid_t& pcr_pid);

  // Decode the Program Association Table to locate the PMT PID
  bool parse_pat(const PacketizedStream& pat, Pid_t& pmt_pid);

  // Decode the Program Map Table to tag PESs
  bool parse_pmt(const PacketizedStream& pmt, StreamMap_t& streams, Pid_t expected_pcr_pid);

  // Extract a contiguous elementary stream from the container PES
  bool depacketize_stream(const PacketizedStream& stream, ElementaryStream& out);

  /// Implementation ///////////////////////////////////////////////////////////

  namespace _detail
  {
    static constexpr uint8_t _TABLE_ID_PAT = 0;
    static constexpr uint8_t _TABLE_ID_PMT = 2;

    bool _parse_table(m::byte_view pkt_iter, uint8_t expected_table_id, m::byte_view& table_data);
  };

  inline bool demux(m::byte_view input, StreamMap_t& out, Pid_t& pcr_pid)
  {
    static constexpr size_t TS_PKT_SIZE = 188;

    uint64_t PCR = UINT64_MAX;
    pcr_pid = UINT16_MAX;

    while (input.size() > 0) {
      if (input.size() < TS_PKT_SIZE) {
        LOG(ERROR) << "invalid input data bytes_left=" << input.size();
        return false;
      }
      m::byte_view pkt_iter(input.data(), TS_PKT_SIZE);
      input.remove_prefix(TS_PKT_SIZE);

      // Parse & verify header fields
      const uint8_t sync_byte             = pkt_iter[0];
      const bool transport_error          = pkt_iter[1] & 0x80;
      const bool payload_start            = pkt_iter[1] & 0x40;
      const bool transport_pri            = pkt_iter[1] & 0x20;
      const Pid_t pid                     = ((pkt_iter[1] & 0x1F) << 8) | pkt_iter[2];
      const uint8_t transport_scrambling  = (pkt_iter[3] >> 6) & 0b11;
      const uint8_t adaptation_field      = (pkt_iter[3] >> 4) & 0b11;
      const uint8_t continuity_counter    = pkt_iter[3] & 0b1111;
      if (sync_byte != 'G' ||
          transport_error ||
          transport_pri ||
          transport_scrambling != 0b00 ||
          adaptation_field == 0b00) {
        LOG(ERROR) << "invalid ts packet";
        return false;
      }
      pkt_iter.remove_prefix(4);

      auto i_stream = out.find(pid);
      if (i_stream == out.end())
        i_stream = out.insert({pid, PacketizedStream()}).first;
      auto& stream = i_stream->second;

      // Parse adaptation field if it exists
      if (adaptation_field == 0b10 || adaptation_field == 0b11) {
        const uint8_t adaptation_len = pkt_iter[0];
        pkt_iter.remove_prefix(1);
        if ((adaptation_field == 0b10 && adaptation_len == 0) ||
            (adaptation_field == 0b11 && adaptation_len >= pkt_iter.size()) ||
            (adaptation_len > pkt_iter.size())) {
          LOG(ERROR) << "invalid adaptation field length";
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
            LOG(ERROR) << "invalid adaptation field flags";
            return false;
          }

          if (pcr_flag) {
            if (adaptation_len != 7) {
              LOG(ERROR) << "invalid adaptation field pcr";
              return false;
            }
            if (not payload_start) {
              LOG(ERROR) << "pcr received in midst of payload unit";
              return false;
            }
            if (pid != pcr_pid && pcr_pid != UINT16_MAX) {
              LOG(ERROR) << "conflicting pcr pids found";
              return false;
            }
            pcr_pid = pid;
            const uint64_t PCR_base =
              (pkt_iter[1] << 25) |
              (pkt_iter[2] << 17) |
              (pkt_iter[3] <<  9) |
              (pkt_iter[4] <<  1) |
              ((pkt_iter[5] >> 7) & 0b1);
            const uint64_t PCR_extension =
              ((uint64_t(pkt_iter[5]) & 0b1) << 8) |
              pkt_iter[6];
            PCR = (PCR_base * 300ul) + PCR_extension;
          }
          else if (random_access) {
            if (adaptation_len != 1) {
              LOG(ERROR) << "invalid adaptation field random_access";
              return false;
            }
            //TODO: Do anything w/ this field?
          }
          else {
            for (uint8_t i = 1; i < adaptation_len; ++i) {
              if (pkt_iter[i] != 0xFF) {
                LOG(ERROR) << "invalid adaptation field stuffing bytes";
                return false;
              }
            }
          }

          pkt_iter.remove_prefix(adaptation_len);
        }
      }

      // Ignore null packets inserted for constant bit rates
      if (pid == PID_NULL)
        continue;

      // Verify sequence number continuity per stream
      if (continuity_counter != ((stream.SeqNum + 1) & 0xF)) {
        LOG(ERROR) << "sequence error for stream pid=" << pid
                   << " expected=" << int((stream.SeqNum + 1) & 0xF)
                   << " received=" << int(continuity_counter);
        return false;
      }
      stream.SeqNum = continuity_counter;

      // Parse packet payload
      if (!(adaptation_field & 0b1)) {
        LOG(ERROR) << "invalid packet, no payload";
        return false;
      }
      else {
        if (payload_start)
          stream.Packets.emplace_back(PacketizedStream::Packet{PCR, m::stream_buf{}});
        else if (stream.Packets.empty()) {
          LOG(ERROR) << "payload not started with no previous packet stream=" << pid;
          return false;
        }
        stream.Packets.back().Data.write(pkt_iter);
      }
    }

    return true;
  }

  inline bool parse_pat(const PacketizedStream& pat, Pid_t& pmt_pid)
  {
    pmt_pid = UINT16_MAX;
    for (const auto& pkt : pat.Packets) {
      m::byte_view table_data;
      if (!_detail::_parse_table(pkt.Data.get_read_view(), _detail::_TABLE_ID_PAT, table_data) ||
          table_data.size() != 4) {
        LOG(ERROR) << "invalid PAT table";
        return false;
      }

      const uint16_t program_num  = (table_data[0] << 8) | table_data[1];
      const uint8_t reserved2     = (table_data[2] >> 5) & 0b111;
      const Pid_t pid             = ((table_data[2] & 0b11111) << 8) | table_data[3];
      if (program_num != 1 ||
          reserved2 != 0b111 ||
          (pmt_pid != UINT16_MAX && pmt_pid != pid)) {
        LOG(ERROR) << "invalid PAT table";
        return false;
      }
      pmt_pid = pid;
    }
    return true;
  }

  inline bool parse_pmt(const PacketizedStream& pmt, StreamMap_t& streams, Pid_t expected_pcr_pid)
  {
    for (const auto& pkt : pmt.Packets) {
      m::byte_view table_data;
      if (!_detail::_parse_table(pkt.Data.get_read_view(), _detail::_TABLE_ID_PMT, table_data))
        return false;

      if (table_data.size() < 4) {
        LOG(ERROR) << "invalid PMT table";
        return false;
      }
      const uint8_t reserved2  = (table_data[0] >> 5) & 0b111;
      const Pid_t pcr_pid      = ((table_data[0] & 0b11111) << 8) | table_data[1];
      const uint8_t reserved3  = (table_data[2] >> 2) & 0b111111;
      const uint16_t pid_len   = ((table_data[2] & 0b11) << 8) | table_data[3];
      if (reserved2 != 0b111 ||
          reserved3 != 0b111100 ||
          pid_len != 0 ||
          pcr_pid != expected_pcr_pid) {
        LOG(ERROR) << "invalid PMT table header";
        return false;
      }
      table_data.remove_prefix(4);

      while (!table_data.empty()) {
        if (table_data.size() < 5) {
          LOG(ERROR) << "invalid PMT table stream data";
          return false;
        }
        const uint8_t stream_type       = table_data[0];
        const uint8_t reserved4         = (table_data[1] >> 5) & 0b111;
        const Pid_t stream_pid          = ((table_data[1] & 0b11111) << 8) | table_data[2];
        const uint8_t reserved5         = (table_data[3] >> 2) & 0b111111;
        const uint16_t stream_info_len  = ((table_data[3] & 0b11) << 8) | table_data[4];
        if (reserved4 != 0b111 ||
            reserved5 != 0b111100 ||
            stream_info_len + 5u > table_data.size()) {
          LOG(ERROR) << "invalid PMT table stream fields";
          return false;
        }
        m::byte_view stream_info(table_data.data() + 5, stream_info_len);
        table_data.remove_prefix(5 + stream_info_len);

        auto i_stream = streams.find(stream_pid);
        if (i_stream == streams.end()) {
          LOG(ERROR) << "unknown stream in PMT pid=" << int(stream_pid);
          return false;
        }
        auto& stream = i_stream->second;
        if (stream.Type != 0 && stream.Type != stream_type) {
          LOG(ERROR) << "conflicting stream types from PMT pid=" << int(stream_pid);
          return false;
        }
        stream.Type = stream_type;

        static constexpr uint8_t DESC_TAG_LANG = 10;

        // Read stream_info descriptors
        while (!stream_info.empty()) {
          if (stream_info.size() < 2) {
            LOG(ERROR) << "invalid PMT stream info descriptor";
            return false;
          }
          const uint8_t desc_tag = stream_info[0];
          const uint8_t desc_len = stream_info[1];
          stream_info.remove_prefix(2);

          if (desc_len > stream_info.size()) {
            LOG(ERROR) << "invalid PMT stream info descriptor";
            return false;
          }

          if (desc_tag == DESC_TAG_LANG) {
            if (stream_info[desc_len-1] != 0x00 && stream_info[desc_len-1] != 0x03) {
              LOG(ERROR) << "invalid PMT stream info lang descriptor";
              return false;
            }
            std::string_view lang((const char*) stream_info.data(), desc_len - 1);
            if (!stream.Lang.empty() && stream.Lang != lang) {
              LOG(ERROR) << "conflicting stream info lang descriptor pid=" << int(stream_pid);
              return false;
            }
            stream.Lang = lang;
          } else {
            LOG(ERROR) << "unhandled stream descriptor tag=" << int(desc_tag);
            return false;
          }
          stream_info.remove_prefix(desc_len);
        }
      }
    }
    return true;
  }

  inline bool depacketize_stream(const PacketizedStream& stream, ElementaryStream& out)
  {
    for (const auto& pkt : stream.Packets) {
      auto pkt_iter = pkt.Data.get_read_view();
      if (pkt_iter.size() < 9) {
        LOG(ERROR) << "invalid pes header";
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
          pkt_iter.size() < 9u + pes_hdr_len) {
        LOG(ERROR) << "invalid pes header";
        return false;
      }
      pkt_iter.remove_prefix(9u);
      const uint64_t PTS = 300ul *
        ((((pkt_iter[0] >> 1) & 0b111) << 30) |
         (pkt_iter[1] << 22) |
         ((((pkt_iter[2] >> 1) & 0b1111111) << 15)) |
         (pkt_iter[3] << 7) |
         ((((pkt_iter[4] >> 1) & 0b1111111))));
      pkt_iter.remove_prefix(5);
      uint64_t DTS = UINT64_MAX;
      if (pts_dts == 0b11) {
        DTS = 300ul *
          ((((pkt_iter[0] >> 1) & 0b111) << 30) |
           (pkt_iter[1] << 22) |
           ((((pkt_iter[2] >> 1) & 0b1111111) << 15)) |
           (pkt_iter[3] << 7) |
           ((((pkt_iter[4] >> 1) & 0b1111111))));
        pkt_iter.remove_prefix(5);
      }
      out.Times[out.Data.get_read_left()] = { pkt.PCR, PTS, DTS };
      out.Data.write(pkt_iter.data(), pkt_iter.size());
    }
    return true;
  }

  inline bool _detail::_parse_table(m::byte_view pkt_iter, uint8_t expected_table_id, m::byte_view& table_data)
  {
    if (pkt_iter.empty()) {
      LOG(ERROR) << "invalid table";
      return false;
    }

    const uint8_t pointer_bytes = pkt_iter[0];
    if (pointer_bytes != 0) {
      LOG(ERROR) << "unhandled table pointer bytes num=" << int(pointer_bytes);
      return false;
    }
    pkt_iter.remove_prefix(1);

    if (pkt_iter.size() < 3) {
      LOG(ERROR) << "invalid table";
      return false;
    }
    const uint8_t table_id          = pkt_iter[0];
    const bool section_syntax_flag  = pkt_iter[1] & 0x80;
    //const bool private_flag         = pkt_iter[1] & 0x40;
    const uint8_t reserved0         = (pkt_iter[1] >> 2) & 0b1111;
    const uint16_t section_len      = ((pkt_iter[1] & 0b11) << 8) | pkt_iter[2];
    if (table_id != expected_table_id ||
        ! section_syntax_flag ||
        reserved0 != 0b1100 ||
        section_len > 1021 ||
        section_len < 9 ||
        section_len > pkt_iter.size()) {
      LOG(ERROR) << "invalid table fields";
      return false;
    }
    pkt_iter.remove_prefix(3);

    const uint16_t data_len         = section_len - 9;
    const uint16_t table_id_ext     = (pkt_iter[0] << 8) | pkt_iter[1];
    const uint8_t reserved1         = (pkt_iter[2] >> 6) & 0b11;
    const uint8_t version           = (pkt_iter[2] >> 1) & 0b11111;
    const bool current_next         = pkt_iter[2] & 0b1;
    const uint8_t section_num       = pkt_iter[3];
    const uint8_t last_section_num  = pkt_iter[4];
    table_data                      = m::byte_view(pkt_iter.data() + 5, data_len);
    /* const uint32_t crc =
      (pkt_iter[5 + data_len + 0] << 24) |
      (pkt_iter[5 + data_len + 1] << 16) |
      (pkt_iter[5 + data_len + 2] << 8)  |
      (pkt_iter[5 + data_len + 3] << 0); */
    if (table_id_ext != 1 ||
        reserved1 != 0b11 ||
        version != 0 ||
        ! current_next ||
        section_num != 0 ||
        last_section_num != 0) {
      LOG(ERROR) << "invalid section fields";
      return false;
    }
    pkt_iter.remove_prefix(section_len);

    // All remaining bytes should be stuffed
    while (!pkt_iter.empty()) {
      if (pkt_iter[0] != 0xFF) {
        LOG(ERROR) << "invalid table stuffing";
        return false;
      }
      pkt_iter.remove_prefix(1);
    }

    return true;
  }

};
