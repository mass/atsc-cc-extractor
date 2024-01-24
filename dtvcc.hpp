#pragma once

#include "mpegts.hpp"

////////////////////////////////////////////////////////////////////////////////
/// Utilities for working with data in the CTA-708 closed captioning format.
/// TODO: Cleanup
namespace dtvcc
{
  // Container for closed captioning data
  struct Packet
  {
    uint8_t Data[39];
    uint8_t Num = 0u;
    mpegts::Timecodes Times;
  };
  static_assert(sizeof(Packet) == 64);

  // TODO
  static inline bool parse_dtvcc_user_data(m::byte_view& iter, std::vector<Packet>& out);

  // Extract contiguous "services" of DTVCC data from the packetized transport stream
  static inline bool depacketize_dtvcc_stream(
      const std::vector<Packet>& dtvcc_packets,
      std::map<uint8_t, mpegts::ElementaryStream>& out);

  // TODO
  static inline bool process_dtvcc_stream(const mpegts::ElementaryStream& dtvcc_stream);

  /// Implementation ///////////////////////////////////////////////////////////

  static inline bool parse_dtvcc_user_data(m::byte_view& iter, std::vector<Packet>& out)
  {
    // user_data header
    uint8_t cc_count = 0;
    {
      if (iter.size() < 3) {
        LOG(ERROR) << "invalid user_data header";
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
        LOG(ERROR) << "invalid user_data header";
        return false;
      }
      iter.remove_prefix(2);
    }

    // Array of cc_data_pkt structures
    for (unsigned i = 0; i < cc_count; ++i)
    {
      if (iter.size() < 3) {
        LOG(ERROR) << "invalid cc_data_pkt";
        return false;
      }
      const uint8_t marker = (iter[0] >> 3) & 0b11111;
      const bool cc_valid = iter[0] & 0x04;
      const uint8_t cc_type = iter[0] & 0b11;
      const uint8_t data1 = iter[1];
      const uint8_t data2 = iter[2];
      if (marker != 0b11111) {
        LOG(ERROR) << "invalid cc_data_pkt";
        return false;
      }
      iter.remove_prefix(3);

      if (cc_type == 0 || cc_type == 1)
        continue; // Backwards-compatible EIA-608 / NTSC

      // Start a new DTVCC packet when instructed to do so
      if (!cc_valid || cc_type == 3) {
        if (out.empty() || out.back().Num > 0) {
          out.emplace_back();
          out.back().Num = 0u;

          //TODO
          //const off_t offset = iter.data() -  mpeg2v_stream.Data.get_read();
          //auto it = mpeg2v_stream.Times.lower_bound(offset);
          //assert(it != mpeg2v_stream.Times.begin());
          //--it;
          //out.back().Times = it->second;
        }
      }

      if (cc_valid) {
        if (out.empty()) {
          LOG(ERROR) << "invalid cc_data_pkt";
          return false;
        }
        auto& pkt = out.back();
        if (pkt.Num + 2u > sizeof(pkt.Data)) {
          LOG(ERROR) << "cc_data_pkt too large";
          return false;
        }
        pkt.Data[pkt.Num++] = data1;
        pkt.Data[pkt.Num++] = data2;
      }
    }

    if (not out.empty() && out.back().Num == 0u)
      out.pop_back();

    return true;
  }

  static inline bool depacketize_dtvcc_stream(
      const std::vector<Packet>& dtvcc_packets,
      std::map<uint8_t, mpegts::ElementaryStream>& out)
  {
    uint8_t prev_seq = UINT8_MAX;
    for (const auto& pkt : dtvcc_packets)
    {
      for (auto& [_, svc] : out)
        svc.Times[svc.Data.get_read_left()] = pkt.Times;

      auto pkt_iter = m::byte_view{pkt.Data, pkt.Num};
      if (pkt_iter.size() < 1) {
        LOG(ERROR) << "invalid dtvcc packet";
        return false;
      }

      const uint8_t seq_num = (pkt_iter[0] >> 6) & 0b11;
      if (seq_num != ((prev_seq + 1) % 4) && prev_seq != UINT8_MAX) {
        LOG(WARN) << "seqnum error received=" << int(seq_num) << " expected=" << int((prev_seq+1)%4);
        //return false;
      }
      prev_seq = seq_num;

      const uint8_t pkt_size_code = pkt_iter[0] & 0b111111;
      const uint8_t pkt_bytes = uint8_t((pkt_size_code == 0 ? 128 : pkt_size_code * 2) - 1);
      pkt_iter.remove_prefix(1);
      if (pkt_bytes != pkt_iter.size()) {
        LOG(ERROR) << "invalid dtvcc packet";
        return false;
      }

      while (!pkt_iter.empty()) {
        const uint8_t svc_num = (pkt_iter[0] >> 5) & 0b111;
        const uint8_t blk_size = pkt_iter[0] & 0b11111;
        pkt_iter.remove_prefix(1);
        if (svc_num == 0 || blk_size == 0) {
          if (not (svc_num == 0 && blk_size == 0 &&
                   (pkt_iter.empty() ||
                    (pkt_iter.size() == 1 && pkt_iter[0] == 0u)))) { //TODO: Why extra 0 ?
          //if (!(svc_num == 0 && blk_size == 0 && pkt_iter.empty())) {
            LOG(ERROR) << "invalid dtvcc packet";
            return false;
          }
          break;
        }
        if (svc_num == 7 || blk_size > pkt_iter.size()) {
          LOG(ERROR) << "invalid dtvcc packet";
          return false;
        }
        out[svc_num].Data.write(pkt_iter.substr(0, blk_size));
        pkt_iter.remove_prefix(blk_size);
      }
    }
    return true;
  }

  static inline bool process_dtvcc_stream(const mpegts::ElementaryStream& dtvcc_stream)
  {
    std::vector<char> buf;

    const auto flush_buf = [&buf] () {
      buf.push_back('\0');
      if (buf.size() > 1)
        LOG(INFO) << "buf=(" << buf.data() << ")";
      buf.clear();
    };

    auto iter = dtvcc_stream.Data.get_read_view();
    while (!iter.empty())
    {
      const uint8_t cmd = iter[0];

      // Check for EXT1 flag, command is in next byte
      if (cmd == 0x10) {
        if (iter.size() == 1) {
          LOG(ERROR) << "invalid dtvcc stream";
          return false;
        }
        const uint8_t ecmd = iter[1];

        // Extended miscellaneous control codes (C2)
        if (ecmd <= 0x1F) {
          // No commands defined (yet)
          if (ecmd <= 0x07)
            iter.remove_prefix(2);
          else if (ecmd <= 0x0F)
            iter.remove_prefix(3);
          else if (ecmd <= 0x17)
            iter.remove_prefix(4);
          else
            iter.remove_prefix(5);
        }
        // Extended control code set 2 (C3)
        else if (ecmd >= 0x80 && ecmd <= 0x9F) {
          // No commands defined (yet)
          if (ecmd <= 0x87)
            iter.remove_prefix(6);
          else if (ecmd <= 0x8F)
            iter.remove_prefix(7);
          else
            iter.remove_prefix(2);
        }
        // Extended control code set 1 (G2)
        else if (ecmd >= 0x20 && ecmd <= 0x7F) {
          //TODO: Printable characters to handle here
          iter.remove_prefix(2);
          LOG(INFO) << "HERE1";
          return false;
        }
        // Future characters and icons (G3)
        else if (ecmd >= 0xA0) {
          //TODO: Handle [CC] icon
          iter.remove_prefix(2);
          LOG(INFO) << "HERE2";
          return false;
        }
        continue;
      }

      // Subset of ASCII control codes (C0)
      if (cmd <= 0x1F) {
        flush_buf();
        if (cmd <= 0x0F) {
          //TODO: Handle codes here
          iter.remove_prefix(1);
          LOG(INFO) << "HERE3 " << int(cmd);
          //return false;
          /*
          switch (cmd) {
            case 0x00: std::cout << "    <C0_CMD_NUL>" << std::endl; break;
            case 0x03: std::cout << "    <C0_CMD_ETX>" << std::endl; break;
            case 0x08: std::cout << "    <C0_CMD_BS>" << std::endl; break;
            case 0x0c: std::cout << "    <C0_CMD_FF>" << std::endl; break;
            case 0x0d: std::cout << "    <C0_CMD_CR>" << std::endl; break;
            case 0x0e: std::cout << "    <C0_CMD_HCR>" << std::endl; break;
            default: //TODO return false;
          } */
        }
        else if (cmd <= 0x17)
          iter.remove_prefix(2);
        else if (cmd <= 0x1F)
          iter.remove_prefix(3);
        continue;
      }
      // Caption control codes (C1)
      else if (cmd >= 0x80 && cmd <= 0x9F) {
        flush_buf();
        if (cmd <= 0x87) {
          //TODO: Handle SetCurrentWindow0-7
          std::cout << "<C1_CMD_CWn>" << std::endl;
          iter.remove_prefix(1);
        }
        else if (cmd >= 0x98 && cmd <= 0x9F) {
          //TODO: Handle DefineWindow0-7
          std::cout << "<C1_CMD_DFn>" << std::endl;
          iter.remove_prefix(7);
        }
        else {
          //TODO: Handle all these
          switch (cmd) {
            case 0x88:
              std::cout << "<C1_CMD_CLW>" << std::endl;
              iter.remove_prefix(2);
              break;
            case 0x89:
              std::cout << "<C1_CMD_DSW>" << std::endl;
              iter.remove_prefix(2);
              break;
            case 0x8A:
              std::cout << "<C1_CMD_HDW>" << std::endl;
              iter.remove_prefix(2);
              break;
            case 0x8B:
              std::cout << "<C1_CMD_TGW>" << std::endl;
              iter.remove_prefix(2);
              break;
            case 0x8C:
              std::cout << "<C1_CMD_DLW>" << std::endl;
              iter.remove_prefix(2);
              break;
            case 0x8D:
              std::cout << "<C1_CMD_DLY>" << std::endl;
              iter.remove_prefix(2);
              break;
            case 0x8E:
              std::cout << "<C1_CMD_DLC>" << std::endl;
              iter.remove_prefix(1);
              break;
            case 0x8F:
              std::cout << "<C1_CMD_RST>" << std::endl;
              iter.remove_prefix(1);
              break;
            case 0x90:
              std::cout << "<C1_CMD_SPA>" << std::endl;
              iter.remove_prefix(3);
              break;
            case 0x91:
              std::cout << "<C1_CMD_SPC>" << std::endl;
              iter.remove_prefix(4);
              break;
            case 0x92:
              std::cout << "<C1_CMD_SPL>" << std::endl;
              iter.remove_prefix(3);
              break;
            case 0x97:
              std::cout << "<C1_CMD_SWA>" << std::endl;
              iter.remove_prefix(5);
              break;
            default:
              //TODO
              LOG(INFO) << "HERE4";
              return false;
          }
        }
        continue;
      }
      // Modified ASCII printable characters (G0)
      else if (cmd >= 0x20 && cmd <= 0x7F) {
        buf.push_back(cmd);
        iter.remove_prefix(1);
        continue;
      }
      // Latin-1 characters (G1)
      else if (cmd >= 0xA0) {
        //TODO: Printable characters to handle here
        iter.remove_prefix(1);
        LOG(INFO) << "HERE5 " << int(cmd);
        //return false;
        continue;
      }

      LOG(ERROR) << "invalid CTA-708 command code=" << int(cmd);
      return false;
    }

    return true;
  }

};
