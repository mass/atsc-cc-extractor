#pragma once

#include <m/stream_buf.hpp>

////////////////////////////////////////////////////////////////////////////////
/// Utilities for working with data in the MPEG2-Video coding format.
namespace mpegv
{
  // DTVCC Packet - container for closed captioning data
  struct DtvccPacket
  {
    uint8_t Data[39];
    uint8_t Num = 0u;
    mpegts::Timecodes Times;
  };
  static_assert(sizeof(DtvccPacket) == 64);

  // Extract DTVCC transport stream (CEA-708 closed captions) from MPEG2 video stream
  bool extract_dtvcc_packets(mpegts::ElementaryStream& mpeg2v_stream, std::vector<DtvccPacket>& out);

  // Extract contiguous "services" of DTVCC data from the packetized transport stream
  bool depacketize_dtvcc_stream(const std::vector<DtvccPacket>& dtvcc_packets,
                                std::map<uint8_t, mpegts::ElementaryStream>& out);

  // TODO
  bool process_dtvcc_stream(const mpegts::ElementaryStream& dtvcc_stream); //TODO: out?

  /// Implementation ///////////////////////////////////////////////////////////

  namespace _detail
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

    static constexpr uint32_t _START_CODE_SEQ = 0x000001B3;
    static constexpr uint32_t _START_CODE_EXT = 0x000001B5;
    static constexpr uint32_t _START_CODE_GOP = 0x000001B8;
    static constexpr uint32_t _START_CODE_PIC = 0x00000100;
    static constexpr uint32_t _START_CODE_USR = 0x000001B2;
    static constexpr uint32_t _START_CODE_SLL = 0x00000101;
    static constexpr uint32_t _START_CODE_SLH = 0x000001AF;

    bool _parse_sequence(mpegts::ElementaryStream& mpeg2v_stream, std::vector<DtvccPacket>& out);
    bool _parse_gop(mpegts::ElementaryStream& mpeg2v_stream, std::vector<DtvccPacket>& out);
    bool _parse_picture(mpegts::ElementaryStream& mpeg2v_stream, std::vector<DtvccPacket>& out, uint16_t& picture_seq);
    bool _parse_slice(mpegts::ElementaryStream& mpeg2v_stream);
    bool _parse_user(mpegts::ElementaryStream& mpeg2v_stream, std::vector<DtvccPacket>& out);
  };

  inline bool extract_dtvcc_packets(mpegts::ElementaryStream& mpeg2v_stream, std::vector<DtvccPacket>& out)
  {
    mpeg2v_stream.Iter = mpeg2v_stream.Data.get_read_view();

    while (mpeg2v_stream.Iter.size() >= 4)
      if (!_detail::_parse_sequence(mpeg2v_stream, out))
        return false;

    if (!out.empty() && out.back().Num == 0u)
      out.pop_back();
    if (out.empty()) {
      LOG(ERROR) << "no dtvcc packets found";
      return false;
    }

    return true;
  }

  inline bool _detail::_parse_sequence(mpegts::ElementaryStream& mpeg2v_stream, std::vector<DtvccPacket>& out)
  {
    auto& iter = mpeg2v_stream.Iter;

    // Sequence header
    {
      if (iter.size() < 12) {
        LOG(ERROR) << "invalid sequence header";
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
      if (start_code != _START_CODE_SEQ ||
          //horz_size != 1920 || TODO
          //vert_size != 1080 || TODO
          aspect_ratio != 0x3 || // 16:9
          //frame_rate != 0x4 || // 29.9 TODO
          //bit_rate TODO
          !reserved0 ||
          //vbv_buf_size TODO
          constrained_params) {
        LOG(ERROR) << "invalid sequence header";
        return false;
      }

      //TODO
      UNUSED(horz_size);
      UNUSED(vert_size);
      UNUSED(frame_rate);
      UNUSED(bit_rate);
      UNUSED(vbv_buf_size);

      size_t req_bytes = 12;
      const bool intra_quantiser = iter[11] & 0x02;
      if (intra_quantiser)
        req_bytes += 64;
      if (iter.size() < req_bytes) {
        LOG(ERROR) << "invalid sequence header";
        return false;
      }
      const bool non_intra_quantiser = iter[req_bytes - 1] & 0x01;
      if (non_intra_quantiser)
        req_bytes += 64;
      if (iter.size() < req_bytes) {
        LOG(ERROR) << "invalid sequence header";
        return false;
      }
      iter.remove_prefix(req_bytes);
    }

    // Sequence extension header
    {
      if (iter.size() < 10) {
        LOG(ERROR) << "invalid sequence extension header";
        return false;
      }
      const uint32_t start_code =
        (iter[0] << 24) |
        (iter[1] << 16) |
        (iter[2] << 8)  |
        (iter[3] << 0);
      const uint8_t variety = (iter[4] >> 4) & 0x0F;
      //TODO: Other fields
      if (start_code != _START_CODE_EXT ||
          variety != 0b0001) {
        LOG(ERROR) << "invalid sequence extension header";
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
      if (start_code == _START_CODE_GOP) {
        if (!_parse_gop(mpeg2v_stream, out))
          return false;
        ++count;
      }
      else if (start_code == _START_CODE_SEQ) {
        break;
      }
      else {
        LOG(ERROR) << "invalid start code in sequence structure";
        return false;
      }
    }

    if (count == 0) {
      LOG(ERROR) << "invalid sequence, no gop structures found";
      return false;
    }

    return true;
  }

  inline bool _detail::_parse_gop(mpegts::ElementaryStream& mpeg2v_stream, std::vector<DtvccPacket>& out)
  {
    auto& iter = mpeg2v_stream.Iter;

    // Group of pictures header
    {
      if (iter.size() < 8) {
        LOG(ERROR) << "invalid gop header";
        return false;
      }
      const uint32_t start_code =
        (iter[0] << 24) |
        (iter[1] << 16) |
        (iter[2] << 8)  |
        (iter[3] << 0);
      /*
      const uint8_t hour = (iter[4] >> 2) & 0b11111;
      const uint8_t minu =
        ((iter[4] & 0b11) << 4) |
        ((iter[5] >> 4) & 0b1111);
      const uint8_t secs =
        ((iter[5] & 0b111) << 3) |
        ((iter[6] >> 5) & 0b111);
      const uint8_t frme =
        ((iter[6] & 0b11111) << 1) |
        ((iter[7] >> 7) & 0b1);
       */

      //TODO: Other fields
      if (start_code != _START_CODE_GOP) {
        LOG(ERROR) << "invalid gop header";
        return false;
      }
      iter.remove_prefix(8);

      //std::cout << "GOP " << int(hour) << ":" << int(minu) << ":" << int(secs) << "." << int(frme) << std::endl;
    }

    // Array of picture structures
    // Note: Pictures are encoded in a different order than they are meant to
    // be displayed. CC data is meant to be processed in display order, not in
    // encoded order, so we must reorder pictures before processing CC packets.
    std::vector<std::vector<DtvccPacket>> pictures;
    while (iter.size() >= 4) {
      const uint32_t start_code =
        (iter[0] << 24) |
        (iter[1] << 16) |
        (iter[2] << 8)  |
        (iter[3] << 0);
      if (start_code == _START_CODE_PIC) {
        uint16_t picture_seq;
        std::vector<DtvccPacket> pkts;
        if (!_parse_picture(mpeg2v_stream, pkts, picture_seq))
          return false;
        if (!pkts.empty() && pkts.back().Num == 0u)
          pkts.pop_back();

        if (picture_seq >= pictures.size())
          pictures.resize(picture_seq + 1);
        else if (not pictures[picture_seq].empty())
          return false; // Duplicate
        pictures[picture_seq] = std::move(pkts);
      }
      else if (start_code == _START_CODE_GOP ||
               start_code == _START_CODE_SEQ) {
        break;
      }
      else {
        LOG(ERROR) << "invalid start code in gop structure";
        return false;
      }
    }

    if (pictures.empty()) {
      LOG(ERROR) << "invalid gop, no picture structures found";
      return false;
    }
    for (auto&& pkts : pictures)
      for (auto&& pkt : pkts)
        out.emplace_back(std::move(pkt));

    return true;
  }

  inline bool _detail::_parse_picture(mpegts::ElementaryStream& mpeg2v_stream, std::vector<DtvccPacket>& out, uint16_t& picture_seq)
  {
    auto& iter = mpeg2v_stream.Iter;

    // Picture header
    {
      if (iter.size() < 8) {
        LOG(ERROR) << "invalid picture header";
        return false;
      }
      const uint32_t start_code =
        (iter[0] << 24) |
        (iter[1] << 16) |
        (iter[2] << 8)  |
        (iter[3] << 0);
      picture_seq = (uint16_t(iter[4]) << 2) | ((iter[5] >> 6) & 0b11);
      //TODO: Other fields
      if (start_code != _START_CODE_PIC) {
        LOG(ERROR) << "invalid picture header";
        return false;
      }
      iter.remove_prefix(8);
      if (iter[0] != 0) //TODO
        iter.remove_prefix(1);
    }

    // Picture coding extension header
    {
      if (iter.size() < 11) {
        LOG(ERROR) << "invalid picture coding extension header";
        return false;
      }
      const uint32_t start_code =
        (iter[0] << 24) |
        (iter[1] << 16) |
        (iter[2] << 8)  |
        (iter[3] << 0);
      const uint8_t variety = (iter[4] >> 4) & 0x0F;
      //TODO: Other fields and checks
      if (start_code != _START_CODE_EXT ||
          variety != 0b1000) {
        LOG(ERROR) << "invalid picture coding extension header";
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
      if (start_code == _START_CODE_USR) {
        if (!_parse_user(mpeg2v_stream, out))
          return false;
        ++count;
      }
      else if (start_code >= _START_CODE_SLL && start_code <= _START_CODE_SLH) {
        if (!_parse_slice(mpeg2v_stream))
          return false;
        ++count;
      }
      else if (start_code == _START_CODE_PIC ||
               start_code == _START_CODE_GOP ||
               start_code == _START_CODE_SEQ) {
        break;
      }
      else {
        LOG(ERROR) << "invalid start code in picture structure";
        return false;
      }
    }

    if (count == 0) {
      LOG(ERROR) << "invalid picture, no user structures nor slices found";
      return false;
    }

    return true;
  }

  inline bool _detail::_parse_slice(mpegts::ElementaryStream& mpeg2v_stream)
  {
    //TODO: Just skipping over macroblock stuff for now
    auto& iter = mpeg2v_stream.Iter;
    iter.remove_prefix(4);
    while (iter.size() > 3) {
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

  inline bool _detail::_parse_user(mpegts::ElementaryStream& mpeg2v_stream, std::vector<DtvccPacket>& out)
  {
    auto& iter = mpeg2v_stream.Iter;

    // User header
    {
      if (iter.size() < 9) {
        LOG(ERROR) << "invalid user header";
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
      if (start_code != _START_CODE_USR) {
        LOG(ERROR) << "invalid user header";
        return false;
      }

      //TODO
      if (atsc_ident == *((uint32_t*)"1GTD")) {
        iter.remove_prefix(8);
        while (iter.size() > 3) {
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
        LOG(ERROR) << "invalid user atsc a53 header";
        return false;
      }
      iter.remove_prefix(9);
    }

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
        continue; // Backwards-compatible CEA-608 / NTSC

      // Start a new DTVCC packet when instructed to do so
      if (!cc_valid || cc_type == 3) {
        if (out.empty() || out.back().Num > 0) {
          out.emplace_back();
          out.back().Num = 0u;

          const off_t offset = iter.data() -  mpeg2v_stream.Data.get_read();
          auto it = mpeg2v_stream.Times.lower_bound(offset);
          //assert(it != mpeg2v_stream.Times.begin());
          --it;
          out.back().Times = it->second;
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

    // user_data footer
    {
      if (iter.size() < 1) {
        LOG(ERROR) << "invalid user_data footer";
        return false;
      }
      const uint8_t marker_bits = iter[0];
      if (marker_bits != 0xFF) {
        LOG(ERROR) << "invalid user_data footer";
        return false;
      }
      iter.remove_prefix(1);
    }

    return true;
  }

  inline bool depacketize_dtvcc_stream(
      const std::vector<DtvccPacket>& dtvcc_packets,
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
        LOG(ERROR) << "seqnum error received=" << int(seq_num) << " expected=" << int((prev_seq+1)%4);
        return false;
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
          if (!(svc_num == 0 && blk_size == 0 && pkt_iter.empty())) {
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

  inline bool process_dtvcc_stream(const mpegts::ElementaryStream& dtvcc_stream)
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
          return false;
        }
        // Future characters and icons (G3)
        else if (ecmd >= 0xA0) {
          //TODO: Handle [CC] icon
          iter.remove_prefix(2);
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
          return false;
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
        return false;
      }

      LOG(ERROR) << "invalid CEA-708 command code=" << int(cmd);
      return false;
    }

    return true;
  }

};
