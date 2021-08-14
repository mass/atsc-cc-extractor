#pragma once

#include "utils.hpp"

////////////////////////////////////////////////////////////////////////////////
/// Utilities for working with data in the MPEG2-Video coding format.
namespace mpegv
{
  // Extract DTVCC transport stream (CEA-708 closed captions) from MPEG2 video stream
  bool extract_dtvcc_packets(const utils::StreamBuffer& mpeg2v_stream, std::vector<utils::StreamBuffer>& out);

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

    bool _parse_sequence(utils::byte_view& iter, std::vector<utils::StreamBuffer>& out);
    bool _parse_gop(utils::byte_view& iter, std::vector<utils::StreamBuffer>& out);
    bool _parse_picture(utils::byte_view& iter, std::vector<utils::StreamBuffer>& out);
    bool _parse_slice(utils::byte_view& iter);
    bool _parse_user(utils::byte_view& iter, std::vector<utils::StreamBuffer>& out);
  };

  inline bool extract_dtvcc_packets(const utils::StreamBuffer& mpeg2v_stream, std::vector<utils::StreamBuffer>& out)
  {
    auto iter = mpeg2v_stream.getReadView();
    while (iter.size() >= 4)
      if (!_detail::_parse_sequence(iter, out))
        return false;

    if (!out.empty() && out.back().getReadLeft() <= 0)
      out.pop_back();
    if (out.empty()) {
      std::cerr << "no dtvcc packets found" << std::endl;
      return false;
    }

    return true;
  }

  inline bool _detail::_parse_sequence(utils::byte_view& iter, std::vector<utils::StreamBuffer>& out)
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
      if (start_code != _START_CODE_SEQ ||
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
      if (start_code != _START_CODE_EXT ||
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
      if (start_code == _START_CODE_GOP) {
        if (!_parse_gop(iter, out))
          return false;
        ++count;
      }
      else if (start_code == _START_CODE_SEQ) {
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

  inline bool _detail::_parse_gop(utils::byte_view& iter, std::vector<utils::StreamBuffer>& out)
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
      if (start_code != _START_CODE_GOP) {
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
      if (start_code == _START_CODE_PIC) {
        if (!_parse_picture(iter, out))
          return false;
        ++count;
      }
      else if (start_code == _START_CODE_GOP ||
               start_code == _START_CODE_SEQ) {
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

  inline bool _detail::_parse_picture(utils::byte_view& iter, std::vector<utils::StreamBuffer>& out)
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
      if (start_code != _START_CODE_PIC) {
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
      if (start_code != _START_CODE_EXT ||
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
      if (start_code == _START_CODE_USR) {
        if (!_parse_user(iter, out))
          return false;
        ++count;
      }
      else if (start_code >= _START_CODE_SLL && start_code <= _START_CODE_SLH) {
        if (!_parse_slice(iter))
          return false;
        ++count;
      }
      else if (start_code == _START_CODE_PIC ||
               start_code == _START_CODE_GOP ||
               start_code == _START_CODE_SEQ) {
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

  inline bool _detail::_parse_slice(utils::byte_view& iter)
  {
    //TODO: Just skipping over macroblock stuff for now
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

  inline bool _detail::_parse_user(utils::byte_view& iter, std::vector<utils::StreamBuffer>& out)
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
      if (start_code != _START_CODE_USR) {
        std::cerr << "invalid user header" << std::endl;
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

};
