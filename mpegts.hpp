#pragma once

#include <u/log.hpp>
#include <u/stream_buf.hpp>

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

  // Various time references
  struct Timecodes
  {
    uint64_t PCR = UINT64_MAX; // Main Program Clock Reference (see demux())
    uint64_t PTS = UINT64_MAX; // When should the data by presented (ref PCR)
    uint64_t DTS = UINT64_MAX; // When should the data be decoded (ref PCR)
  };

  // Carries a single video/audio/data stream in variable-length packets
  struct PacketizedElementaryStream
  {
    struct Packet
    {
      uint64_t PCR;
      u::stream_buf<> Data;
    };
    std::vector<Packet> Packets;

    uint8_t SeqNum = 0xF;
    uint8_t Type = 0;
    std::string Lang;
  };
  using StreamMap_t = std::unordered_map<Pid_t, PacketizedElementaryStream>;

  // Elementary Stream - entire stream in one contiguous region of memory
  struct ElementaryStream
  {
    u::stream_buf<> Data;
    std::map<off_t, Timecodes> Times;

    u::byte_view Iter;
  };

  // Demultiplex bytes in MPEG-TS container format into a PES map
  static inline bool demux(u::byte_view input, StreamMap_t& out, Pid_t& pcr_pid);

  // Decode the Program Association Table to locate the PMT PID
  static inline bool parse_pat(const PacketizedElementaryStream& pat, Pid_t& pmt_pid);

  // Decode the Program Map Table to tag PESs
  static inline bool parse_pmt(const PacketizedElementaryStream& pmt, StreamMap_t& streams, Pid_t expected_pcr_pid);

  // Extract a contiguous elementary stream from the container PES
  static inline bool depacketize_stream(const PacketizedElementaryStream& pes, ElementaryStream& out);

  /// Implementation ///////////////////////////////////////////////////////////

  static inline bool demux(u::byte_view input, StreamMap_t& out, Pid_t& pcr_pid)
  {
    // An MPEG TS is nothing but a sequence of fixed-size packets
    static constexpr size_t TS_PKT_SIZE = 188;

    // PCR timestamps are injected into the transport stream periodically,
    // always as part of a single PID. This is the main 27MHz clock that is
    // used to synchonize all the various elementary streams together. A
    // timestamp for each byte can be calculated by linearly interpolating
    // between successive PCR timestamps.
    uint64_t PCR = UINT64_MAX;
    pcr_pid = UINT16_MAX;

    for (size_t pkt_idx = 0; input.size() > 0; ++pkt_idx)
    {
      if (input.size() < TS_PKT_SIZE) {
        LOG(ERROR) << "invalid input data bytes_left=" << input.size();
        return false;
      }
      auto pkt_iter = input.substr(0, TS_PKT_SIZE);
      input.remove_prefix(TS_PKT_SIZE);

      // Parse & verify header fields (ISO 13818-1 Table 2-2)
      const uint8_t sync_byte             = pkt_iter[0];
      const bool transport_error          = pkt_iter[1] & 0x80;
      const bool payload_start            = pkt_iter[1] & 0x40;
      const bool transport_pri            = pkt_iter[1] & 0x20;
      const Pid_t pid                     = ((pkt_iter[1] & 0x1F) << 8) | pkt_iter[2];
      const uint8_t transport_scrambling  = (pkt_iter[3] >> 6) & 0b11;
      const uint8_t adaptation_field      = (pkt_iter[3] >> 4) & 0b11;
      const uint8_t continuity_counter    = pkt_iter[3] & 0b1111;
      if (sync_byte != 0x47 ||
          transport_error ||
          transport_pri ||
          transport_scrambling != 0b00 ||
          (adaptation_field == 0b00 || adaptation_field == 0b10)) {
        LOG(ERROR) << "invalid mpegts header";
        return false;
      }
      pkt_iter.remove_prefix(4);

      // Parse adaptation field if it exists (ISO 13818-1 Table 2-6)
      if (adaptation_field == 0b11) {
        const uint8_t adaptation_len = pkt_iter[0];
        pkt_iter.remove_prefix(1);
        if (adaptation_len >= pkt_iter.size()) {
          LOG(ERROR) << "invalid adaptation field length";
          return false;
        }
        if (adaptation_len > 0) {
          auto adapt_iter = pkt_iter.substr(0, adaptation_len);
          pkt_iter.remove_prefix(adaptation_len);

          const bool discontinuity         = adapt_iter[0] & 0x80;
          const bool random_access         = adapt_iter[0] & 0x40;
          const bool stream_pri            = adapt_iter[0] & 0x20;
          const bool pcr_flag              = adapt_iter[0] & 0x10;
          const bool opcr_flag             = adapt_iter[0] & 0x08;
          const bool splicing_point        = adapt_iter[0] & 0x04;
          const bool transport_private     = adapt_iter[0] & 0x02;
          const bool adaptation_extension  = adapt_iter[0] & 0x01;
          if (discontinuity ||
              stream_pri ||
              opcr_flag ||
              splicing_point ||
              transport_private ||
              adaptation_extension) {
            LOG(ERROR) << "invalid adaptation field flags";
            return false;
          }
          adapt_iter.remove_prefix(1);

          UNUSED(random_access); // NB: Not doing anything with this field

          if (pcr_flag) {
            if (adapt_iter.size() < 6) {
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
            const uint64_t PCR_base = // 33-bits
              (adapt_iter[0] << 25) |
              (adapt_iter[1] << 17) |
              (adapt_iter[2] <<  9) |
              (adapt_iter[3] <<  1) |
              ((adapt_iter[4] >> 7) & 0b1);
            const uint64_t PCR_extension = // 9-bits
              ((adapt_iter[4] & 0b1) << 8) |
              adapt_iter[5];
            PCR = (PCR_base * 300ul) + PCR_extension;
            adapt_iter.remove_prefix(6);
          }

          for (auto stuffing_byte : adapt_iter) {
            if (stuffing_byte != 0xFF) {
              LOG(ERROR) << "invalid adaptation field stuffing bytes";
              return false;
            }
          }
        }
      }

      // Ignore null packets inserted for constant bit rates
      if (pid == PID_NULL)
        continue;

      auto i_pes = out.find(pid);
      if (i_pes == out.end())
        i_pes = out.try_emplace(pid, PacketizedElementaryStream()).first;
      auto& pes = i_pes->second;

      // Verify sequence number continuity per stream
      if (continuity_counter != ((pes.SeqNum + 1) & 0xF)) {
        LOG(ERROR) << "sequence error for stream pid=" << pid
                   << " expected=" << int((pes.SeqNum + 1) & 0xF)
                   << " received=" << int(continuity_counter);
        return false;
      }
      pes.SeqNum = continuity_counter;

      // Ignore all packets before the first PCR we encounter
      if (PCR == UINT64_MAX)
        continue;

      // Save packet payload
      if (payload_start)
        pes.Packets.emplace_back(PacketizedElementaryStream::Packet{PCR, u::stream_buf{}});
      else if (pes.Packets.empty()) {
        LOG(ERROR) << "payload not started with no previous packet stream=" << pid;
        return false;
      }
      pes.Packets.back().Data.write(pkt_iter);
    }

    return true;
  }

  static inline bool parse_pat(const PacketizedElementaryStream& pat, Pid_t& pmt_pid)
  {
    pmt_pid = UINT16_MAX;
    for (const auto& pkt : pat.Packets)
    {
      auto pkt_iter = pkt.Data.get_read_view();

      // Parse the pointer prefix
      if (pkt_iter.size() < 1) {
        LOG(ERROR) << "invalid PAT size";
        return false;
      }
      const uint8_t pointer_bytes = pkt_iter[0];
      if (pointer_bytes != 0) {
        LOG(ERROR) << "invalid PAT pointer";
        return false;
      }
      pkt_iter.remove_prefix(1);

      // Parse the PAT header (ISO 13818-1 Table 2-30)
      // NB: section_len includes the 5 header bytes following it, 4 CRC32
      //   bytes, and the 4 bytes that make up the program number and PMT PID.
      if (pkt_iter.size() < 8) {
        LOG(ERROR) << "invalid PAT size";
        return false;
      }
      const uint8_t table_id              = pkt_iter[0];
      const bool section_syntax_flag      = pkt_iter[1] & 0x80;
      const bool zero                     = pkt_iter[1] & 0x40;
      const uint8_t reserved0             = (pkt_iter[1] >> 4) & 0b11;
      const uint8_t section_len_zero      = (pkt_iter[1] >> 2) & 0b11;
      const uint16_t section_len          = ((pkt_iter[1] & 0b11) << 8) | pkt_iter[2];
      const uint16_t transport_stream_id  = (pkt_iter[3] << 8) | pkt_iter[4];
      const uint8_t reserved1             = (pkt_iter[5] >> 6) & 0b11;
      const uint8_t version_number        = (pkt_iter[5] >> 1) & 0b11111;
      const uint8_t current_next_indc     = pkt_iter[5] & 0b1;
      const uint8_t section_number        = pkt_iter[6];
      const uint8_t last_section_number   = pkt_iter[7];
      if (table_id != 0u ||
          !section_syntax_flag ||
          zero ||
          reserved0 != 0b11 ||
          section_len_zero ||
          section_len != 4u + 5u + 4u ||
          transport_stream_id != 1u ||
          reserved1 != 0b11 ||
          version_number ||
          !current_next_indc ||
          section_number ||
          last_section_number) {
        LOG(ERROR) << "invalid PAT header";
        return false;
      }
      pkt_iter.remove_prefix(8);

      // Parse the PAT section data
      if (pkt_iter.size() < 8) {
        LOG(ERROR) << "invalid PAT size";
        return false;
      }
      const uint16_t program_num  = (pkt_iter[0] << 8) | pkt_iter[1];
      const uint8_t reserved2     = (pkt_iter[2] >> 5) & 0b111;
      const Pid_t pid             = ((pkt_iter[2] & 0b11111) << 8) | pkt_iter[3];
      if (program_num != 1u ||
          reserved2 != 0b111 ||
          (pmt_pid != UINT16_MAX && pmt_pid != pid)) {
        LOG(ERROR) << "invalid PAT table";
        return false;
      }
      pkt_iter.remove_prefix(8); // CRC32 not verified
      pmt_pid = pid; // Extract the PID of the PMT

      // Verify the remainder of the packet is properly stuffed
      for (auto stuffing_byte : pkt_iter) {
        if (stuffing_byte != 0xFF) {
          LOG(ERROR) << "invalid PAT stuffing bytes";
          return false;
        }
      }
    }
    return true;
  }

  static inline bool parse_pmt(const PacketizedElementaryStream& pmt, StreamMap_t& streams, Pid_t expected_pcr_pid)
  {
    for (const auto& pkt : pmt.Packets)
    {
      auto pkt_iter = pkt.Data.get_read_view();

      // Parse the pointer prefix
      if (pkt_iter.size() < 1) {
        LOG(ERROR) << "invalid PMT size";
        return false;
      }
      const uint8_t pointer_bytes = pkt_iter[0];
      if (pointer_bytes != 0) {
        LOG(ERROR) << "invalid PMT pointer";
        return false;
      }
      pkt_iter.remove_prefix(1);

      // Parse the PMT header (ISO 13818-1 Table 2-33)
      // NB: section_len includes the 9 header bytes following it, 4 CRC32
      //   bytes, and then all the bytes for the stream info data.
      if (pkt_iter.size() < 12) {
        LOG(ERROR) << "invalid PMT size";
        return false;
      }
      const uint8_t table_id              = pkt_iter[0];
      const bool section_syntax_flag      = pkt_iter[1] & 0x80;
      const bool zero                     = pkt_iter[1] & 0x40;
      const uint8_t reserved0             = (pkt_iter[1] >> 4) & 0b11;
      const uint8_t section_len_zero      = (pkt_iter[1] >> 2) & 0b11;
      const uint16_t section_len          = ((pkt_iter[1] & 0b11) << 8) | pkt_iter[2];
      const uint16_t program_number       = (pkt_iter[3] << 8) | pkt_iter[4];
      const uint8_t reserved1             = (pkt_iter[5] >> 6) & 0b11;
      const uint8_t version_number        = (pkt_iter[5] >> 1) & 0b11111;
      const uint8_t current_next_indc     = pkt_iter[5] & 0b1;
      const uint8_t section_number        = pkt_iter[6];
      const uint8_t last_section_number   = pkt_iter[7];
      const uint8_t reserved2             = (pkt_iter[8] >> 5) & 0b111;
      const Pid_t pcr_pid                 = ((pkt_iter[8] & 0b11111) << 8) | pkt_iter[9];
      const uint8_t reserved3             = (pkt_iter[10] >> 4) & 0b1111;
      const uint8_t prog_info_len_zero    = (pkt_iter[10] >> 2) & 0b11;
      const uint16_t prog_info_len        = ((pkt_iter[10] & 0b11) << 8) | pkt_iter[11];
      if (table_id != 2u ||
          !section_syntax_flag ||
          zero ||
          reserved0 != 0b11 ||
          section_len_zero ||
          (section_len < 9 + 4 || section_len > 0x3FD) ||
          program_number != 1u ||
          reserved1 != 0b11 ||
          version_number ||
          !current_next_indc ||
          section_number ||
          last_section_number ||
          reserved2 != 0b111 ||
          pcr_pid != expected_pcr_pid ||
          reserved3 != 0b1111 ||
          prog_info_len_zero ||
          prog_info_len) {
        LOG(ERROR) << "invalid PMT header";
        return false;
      }
      pkt_iter.remove_prefix(12);

      uint16_t table_data_len = section_len - 9 - 4;
      if (table_data_len + 4u > pkt_iter.size()) {
        LOG(ERROR) << "invalid PMT size";
        return false;
      }
      auto table_data = pkt_iter.substr(0, table_data_len);
      pkt_iter.remove_prefix(table_data_len);
      pkt_iter.remove_prefix(4); // CRC32 not verified

      while (!table_data.empty())
      {
        // Parse each stream data block (ISO 13818-1 Table 2-33)
        if (table_data.size() < 5) {
          LOG(ERROR) << "invalid PMT stream data";
          return false;
        }
        const uint8_t stream_type           = table_data[0];
        const uint8_t reserved4             = (table_data[1] >> 5) & 0b111;
        const Pid_t stream_pid              = ((table_data[1] & 0b11111) << 8) | table_data[2];
        const uint8_t reserved5             = (table_data[3] >> 4) & 0b1111;
        const uint8_t stream_info_len_zero  = (table_data[3] >> 2) & 0b11;
        const uint16_t stream_info_len      = ((table_data[3] & 0b11) << 8) | table_data[4];
        if (reserved4 != 0b111 ||
            reserved5 != 0b1111 ||
            stream_info_len_zero ||
            stream_info_len + 5u > table_data.size()) {
          LOG(ERROR) << "invalid PMT stream fields";
          return false;
        }
        table_data.remove_prefix(5);

        // Find the stream with the given PID and set the type
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

        // Parse stream_info descriptors
        auto stream_info = table_data.substr(0, stream_info_len);
        table_data.remove_prefix(stream_info_len);
        while (!stream_info.empty())
        {
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

          static constexpr uint8_t DESC_TAG_LANG      = 0x0A; // ISO639 Language
          static constexpr uint8_t DESC_TAG_PRIV_FMT  = 0x05; // Private format registration

          if (desc_tag == DESC_TAG_LANG) {
            if (stream_info[desc_len-1] != 0x00 && // Undefined
                stream_info[desc_len-1] != 0x03) { // Visual impaired commentary
              LOG(ERROR) << "invalid PMT stream info lang descriptor";
              return false;
            }
            auto lang = u::view<char>(stream_info.substr(0, desc_len - 1));
            if (!stream.Lang.empty() && stream.Lang != lang) {
              LOG(ERROR) << "conflicting stream info lang descriptor pid=" << int(stream_pid);
              return false;
            }
            stream.Lang = lang;
          } else if (desc_tag == DESC_TAG_PRIV_FMT) {
            // NB: These are ignored
          } else {
            LOG(ERROR) << "unhandled stream descriptor tag=" << int(desc_tag);
            return false;
          }
          stream_info.remove_prefix(desc_len);
        }
      }

      // Verify the remainder of the packet is properly stuffed
      for (auto stuffing_byte : pkt_iter) {
        if (stuffing_byte != 0xFF) {
          LOG(ERROR) << "invalid PMT stuffing bytes";
          return false;
        }
      }
    }
    return true;
  }

  static inline bool depacketize_stream(const PacketizedElementaryStream& pes, ElementaryStream& out)
  {
    for (const auto& pkt : pes.Packets)
    {
      auto pkt_iter = pkt.Data.get_read_view();

      // Parse PES packet header (ISO 13818-1 Table 2-21)
      if (pkt_iter.size() < 9) {
        LOG(ERROR) << "invalid pes header";
        return false;
      }
      const uint32_t start_code_prefix  = (pkt_iter[0] << 16) | (pkt_iter[1] << 8) | pkt_iter[2];
      const uint8_t stream_id           = pkt_iter[3];
      const uint16_t pes_pkt_len        = (pkt_iter[4] << 8) | pkt_iter[5];
      const uint8_t one_zero            = (pkt_iter[6] >> 6) & 0b11;
      const uint8_t scrambling_control  = (pkt_iter[6] >> 4) & 0b11;
      const bool priority               = pkt_iter[6] & 0x08;
      const bool alignment_ind          = pkt_iter[6] & 0x04;
      const bool copyright              = pkt_iter[6] & 0x02;
      const bool original_or_copy       = pkt_iter[6] & 0x01;
      const uint8_t pts_dts             = (pkt_iter[7] >> 6) & 0b11;
      const bool escr_flag              = pkt_iter[7] & 0x20;
      const bool es_rate                = pkt_iter[7] & 0x10;
      const bool dsm_trick_mode         = pkt_iter[7] & 0x08;
      const bool additional_copy        = pkt_iter[7] & 0x04;
      const bool crc_flag               = pkt_iter[7] & 0x02;
      const bool ext_flag               = pkt_iter[7] & 0x01;
      const uint8_t pes_hdr_len         = pkt_iter[8];
      if (start_code_prefix != 0x000001 ||
          stream_id != 0xE0 || // (ISO 13818-1 Table 2-22)
          pes_pkt_len || // 0 means not specified
          one_zero != 0b10 ||
          scrambling_control ||
          priority ||
          alignment_ind ||
          copyright ||
          original_or_copy ||
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

      // Parse PTS value (always present)
      const uint64_t PTS = 300ul *
        ((((pkt_iter[0] >> 1) & 0b111) << 30) |
         (pkt_iter[1] << 22) |
         ((((pkt_iter[2] >> 1) & 0b1111111) << 15)) |
         (pkt_iter[3] << 7) |
         ((((pkt_iter[4] >> 1) & 0b1111111))));
      const uint8_t mark0 = (pkt_iter[0] >> 4) & 0b1111;
      const uint8_t mark1 = pkt_iter[0] & 0x01;
      const uint8_t mark2 = pkt_iter[2] & 0x01;
      const uint8_t mark3 = pkt_iter[4] & 0x01;
      if (mark0 != pts_dts ||
          !mark1 ||
          !mark2 ||
          !mark3) {
        LOG(ERROR) << "invalid pes PTS";
        return false;
      }
      pkt_iter.remove_prefix(5);

      // Parse DTS value if present
      uint64_t DTS = PTS;
      if (pts_dts == 0b11) {
        DTS = 300ul *
          ((((pkt_iter[0] >> 1) & 0b111) << 30) |
           (pkt_iter[1] << 22) |
           ((((pkt_iter[2] >> 1) & 0b1111111) << 15)) |
           (pkt_iter[3] << 7) |
           ((((pkt_iter[4] >> 1) & 0b1111111))));
        const uint8_t dmark0 = (pkt_iter[0] >> 4) & 0b1111;
        const uint8_t dmark1 = pkt_iter[0] & 0x01;
        const uint8_t dmark2 = pkt_iter[2] & 0x01;
        const uint8_t dmark3 = pkt_iter[4] & 0x01;
        if (dmark0 != 0b0001 ||
            !dmark1 ||
            !dmark2 ||
            !dmark3) {
          LOG(ERROR) << "invalid pes DTS";
          return false;
        }
        pkt_iter.remove_prefix(5);
      }

      //TODO: Interpolate PCR via MPEGTS packets correctly
      out.Times[out.Data.get_read_left()] = { pkt.PCR, PTS, DTS };
      out.Data.write(pkt_iter.data(), pkt_iter.size());
    }
    return true;
  }

};
