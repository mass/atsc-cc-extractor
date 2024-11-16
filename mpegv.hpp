#pragma once

#include "dtvcc.hpp"
#include "mpegts.hpp"

#include <optional>

////////////////////////////////////////////////////////////////////////////////
/// Utilities for working with data in the MPEG2-Video coding format.
namespace mpegv
{
  // Extract DTVCC transport stream (CTA-708 closed captions) from MPEG2 video stream
  static inline bool extract_dtvcc_packets(mpegts::ElementaryStream& mpeg2v_stream, std::vector<dtvcc::Packet>& out);

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

    static constexpr uint32_t _START_CODE_PREFIX = 0x000001;
    static constexpr uint8_t _START_CODE_SEQ = 0xB3;
    static constexpr uint8_t _START_CODE_EXT = 0xB5;
    static constexpr uint8_t _START_CODE_GOP = 0xB8;
    static constexpr uint8_t _START_CODE_PIC = 0x00;
    static constexpr uint8_t _START_CODE_USR = 0xB2;
    static constexpr uint8_t _START_CODE_SLL = 0x01;
    static constexpr uint8_t _START_CODE_SLH = 0xAF;

    static inline std::optional<uint8_t> _next_start_code(u::byte_view& iter, bool consume);
    static inline void _skipto_start_code(u::byte_view& iter);
    static inline void _skipto_seq_gop(u::byte_view& iter, bool gop);

    static inline bool _parse_sequence(mpegts::ElementaryStream& mpeg2v_stream, std::vector<dtvcc::Packet>& out);
    static inline bool _parse_gop(mpegts::ElementaryStream& mpeg2v_stream, std::vector<dtvcc::Packet>& out);
    static inline bool _parse_picture(
        mpegts::ElementaryStream& mpeg2v_stream, std::vector<dtvcc::Packet>& out, uint16_t& picture_seq);
    static inline bool _parse_slice(mpegts::ElementaryStream& mpeg2v_stream);
    static inline bool _parse_user(mpegts::ElementaryStream& mpeg2v_stream, std::vector<dtvcc::Packet>& out);
  };

  static inline bool extract_dtvcc_packets(mpegts::ElementaryStream& mpeg2v_stream, std::vector<dtvcc::Packet>& out)
  {
    mpeg2v_stream.Iter = mpeg2v_stream.Data.get_read_view();

    // NB: Expectation is that the iter is positioned at a sequence start code
    // every time we enter the loop body. Usually this follows a fully parsed,
    // error free sequence, but if errors were encountered we skip ahead instead.
    while (not mpeg2v_stream.Iter.empty())
      if (not _detail::_parse_sequence(mpeg2v_stream, out))
        _detail::_skipto_seq_gop(mpeg2v_stream.Iter, false);

    // NB: The specs allows for a sequence_end start code with value 0xB7, but
    // I have never observed one so there is no handling for it currently.

    if (out.empty()) {
      LOG(ERROR) << "no dtvcc packets found";
      return false;
    }

    return true;
  }

  // Finds the next start code in the stream, usually already positioned at
  // index zero. Is allowed to skip over any number of zero-value "stuffing"
  // bytes. Any other byte value encountered is an error. Either leaves the
  // iterator positioned at the start code or just past it.
  // NB: Start codes are always byte aligned
  static inline std::optional<uint8_t> _detail::_next_start_code(u::byte_view& iter, bool consume)
  {
    while (iter.size() >= 4) {
      const uint32_t prefix = (iter[0] << 16) | (iter[1] << 8) | (iter[2]);
      if (prefix == _START_CODE_PREFIX) {
        const uint8_t start_code = iter[3];
        if (consume)
          iter.remove_prefix(4);
        return start_code;
      }
      if (iter[0] != 0u) {
        LOG(ERROR) << "nonzero start code stuffing bytes";
        return std::nullopt;
      }
      iter.remove_prefix(1);
    }
    return std::nullopt;
  }

  // Position the iterator at the next start code (but do not consume it). Is
  // allowed to skip over bytes with any value. This is used to skip past
  // structures we aren't currently parsing, and is not an error.
  static inline void _detail::_skipto_start_code(u::byte_view& iter)
  {
    while (iter.size() >= 4) {
      const uint32_t prefix = (iter[0] << 16) | (iter[1] << 8) | (iter[2]);
      if (prefix == _START_CODE_PREFIX)
        return;
      iter.remove_prefix(1);
    }
    // No start codes were found and not enough data left for one
    iter.remove_prefix(iter.size());
  }

  // Position the iterator at the next sequence or GOP (if allowed) start code.
  // This is used when we have some kind of corruption and we are skipping
  // ahead to the next position we can start processing from. Logs details
  // about how large the skipped region was.
  //TODO: Better logging around the time that was skipped, etc.
  static inline void _detail::_skipto_seq_gop(u::byte_view& iter, bool gop)
  {
    auto orig_size = iter.size();
    while (iter.size() >= 4) {
      const uint32_t prefix = (iter[0] << 16) | (iter[1] << 8) | (iter[2]);
      if (prefix == _START_CODE_PREFIX) {
        if (iter[3] == _START_CODE_SEQ || (gop &&
            iter[3] == _START_CODE_GOP)) {
          LOG(WARN) << "skipped bytes=" << orig_size - iter.size();
          return;
        }
        iter.remove_prefix(4);
      } else
        iter.remove_prefix(1);
    }
    // No start codes were found and not enough data left for one
    iter.remove_prefix(iter.size());
    LOG(WARN) << "skipped bytes=" << orig_size;
  }

  static inline bool _detail::_parse_sequence(mpegts::ElementaryStream& mpeg2v_stream, std::vector<dtvcc::Packet>& out)
  {
    auto& iter = mpeg2v_stream.Iter;

    // Parse sequence header (ISO 13818-2 6.2.2.1 / 6.3.3)
    {
      if (_next_start_code(iter, true).value_or(0xFF) != _START_CODE_SEQ) {
        LOG(ERROR) << "invalid sequence header start code";
        return false;
      }
      if (iter.size() < 8) {
        LOG(ERROR) << "invalid sequence header";
        return false;
      }
      const uint16_t horz_size       = (iter[0] << 4) | ((iter[1] >> 4) & 0b1111);
      const uint16_t vert_size       = ((iter[1] & 0b1111) << 8) | iter[2];
      const uint8_t aspect_ratio     = (iter[3] >> 4) & 0b1111;
      const uint8_t frame_rate       = (iter[3] & 0b1111);
      const uint32_t bit_rate        = (iter[4] << 10) | (iter[5] << 2) | ((iter[6] >> 6) & 0b11);
      const uint8_t marker           = (iter[6] >> 5) & 0b1;
      const uint16_t vbv_buf_size    = ((iter[6] & 0b11111) << 5) | ((iter[7] >> 3) & 0b11111);
      const uint8_t constrained      = (iter[7] >> 2) & 0b1;
      const uint8_t load_intra_q     = (iter[7] >> 1) & 0b1;
      // NB: See ATSC A/53-4 Tables 6.1 / 6.2 for allowed formats
      if ((horz_size != 1920 && horz_size != 1280 && horz_size != 720) ||
          (vert_size != 1080 && vert_size != 720  && vert_size != 480) ||
          aspect_ratio != 0x3 ||  // 16:9 (ISO 13818-2 Table 6-3)
          frame_rate != 0x4 ||    // 29.97 (ISO 13818-2 Table 6-4)
          !marker ||
          vbv_buf_size > 488 ||
          constrained) {
        LOG(ERROR) << "invalid sequence header";
        return false;
      }
      UNUSED(bit_rate);
      iter.remove_prefix(7); // NB: The last bit of iter[7] is still unprocessed

      // If the quantiser matrices are present, skip over them
      if (load_intra_q) {
        if (iter.size() < 65) {
          LOG(ERROR) << "invalid sequence header";
          return false;
        }
        iter.remove_prefix(64);
      }
      const uint8_t load_nonintra_q = (iter[0] & 0b1);
      iter.remove_prefix(1);
      if (load_nonintra_q) {
        if (iter.size() < 65) {
          LOG(ERROR) << "invalid sequence header";
          return false;
        }
        iter.remove_prefix(64);
      }
    }

    // Parse sequence extension header (ISO 13818-2 6.2.2.3 / 6.3.5)
    {
      if (_next_start_code(iter, true).value_or(0xFF) != _START_CODE_EXT) {
        LOG(ERROR) << "invalid sequence extension header start code";
        return false;
      }
      if (iter.size() < 6) {
        LOG(ERROR) << "invalid sequence extension header";
        return false;
      }
      const uint8_t ext_id         = (iter[0] >> 4) & 0b1111;
      const uint8_t prof_level     = ((iter[0] & 0b1111) << 4) | ((iter[1] >> 4) & 0b1111);
      const uint8_t progressive    = (iter[1] >> 3) & 0b1;
      const uint8_t chroma_format  = (iter[1] >> 1) & 0b11;
      const uint8_t hsize_ext      = ((iter[1] & 0b1) << 1) | ((iter[2] >> 7) & 0b1);
      const uint8_t vsize_ext      = (iter[2] >> 5) & 0b11;
      const uint8_t bitrate_ext    = ((iter[2] & 0b11111) << 7) | ((iter[3] >> 1) & 0b1111111);
      const uint8_t marker         = (iter[3] & 0b1);
      const uint8_t vbv_sz_ext     = (iter[4]);
      const uint8_t low_delay      = (iter[5] >> 7) & 0b1;
      const uint8_t fr_ext_n       = (iter[5] >> 5) & 0b11;
      const uint8_t fr_ext_d       = (iter[5] & 0b11111);
      if (ext_id != 0b0001 ||
          chroma_format != 0b01 || // ISO 13818-2 Table 6-5
          hsize_ext ||
          vsize_ext ||
          bitrate_ext ||
          !marker ||
          vbv_sz_ext ||
          low_delay ||
          fr_ext_n ||
          fr_ext_d) {
        LOG(ERROR) << "invalid sequence extension header";
        return false;
      }
      UNUSED(prof_level);
      UNUSED(progressive);
      iter.remove_prefix(6);
    }

    // Parse array of group of pictures structures
    {
      std::optional<uint8_t> start_code;
      while ((start_code = _next_start_code(iter, false)) == _START_CODE_GOP)
        if (not _parse_gop(mpeg2v_stream, out))
          _detail::_skipto_seq_gop(mpeg2v_stream.Iter, true);
      if (not iter.empty() &&
          start_code != _START_CODE_SEQ) {
        LOG(ERROR) << "invalid start code in sequence structure";
        return false;
      }
    }

    return true;
  }

  static inline bool _detail::_parse_gop(mpegts::ElementaryStream& mpeg2v_stream, std::vector<dtvcc::Packet>& out)
  {
    auto& iter = mpeg2v_stream.Iter;

    // Parse group of pictures header (ISO 13818-2 6.2.2.6 / 6.3.8)
    // NB: The time_code field is sometimes not populated, and generally not
    // useful for decoding anyway.
    {
      if (_next_start_code(iter, true).value_or(0xFF) != _START_CODE_GOP) {
        LOG(ERROR) << "invalid gop header start code";
        return false;
      }
      if (iter.size() < 4) {
        LOG(ERROR) << "invalid gop header";
        return false;
      }
      //const uint8_t drop_frame_flag  = (iter[0] >> 7) & 0b1;
      //const uint8_t hour             = (iter[0] >> 2) & 0b11111;
      //const uint8_t minu             = ((iter[0] & 0b11) << 4) | ((iter[1] >> 4) & 0b1111);
      const uint8_t mark               = (iter[1] >> 3) & 0b1;
      //const uint8_t secs             = ((iter[1] & 0b111) << 3) | ((iter[2] >> 5) & 0b111);
      //const uint8_t frme             = ((iter[2] & 0b11111) << 1) | ((iter[3] >> 7) & 0b1);
      //const uint8_t closed_gop       = (iter[3] >> 6) & 0b1;
      const uint8_t broken_link        = (iter[3] >> 5) & 0b1;
      if (not mark ||
          broken_link) {
        LOG(ERROR) << "invalid gop header";
        return false;
      }
      iter.remove_prefix(4);
    }

    // Parse array of picture structures
    // NB: Pictures are encoded in a different order than they are meant to
    // be displayed. CC data is meant to be processed in display order, not in
    // encoded order, so we must reorder pictures before processing CC packets.
    std::optional<uint8_t> start_code;
    std::map<uint16_t, std::vector<dtvcc::Packet>> pictures;
    while ((start_code = _next_start_code(iter, false)) == _START_CODE_PIC)
    {
      uint16_t picture_seq;
      std::vector<dtvcc::Packet> pkts;
      if (not _parse_picture(mpeg2v_stream, pkts, picture_seq))
        return false;
      if (not pictures.try_emplace(picture_seq, std::move(pkts)).second) {
        LOG(ERROR) << "duplicate picture seq";
        return false; // Duplicate
      }
    }
    if (not iter.empty() &&
        start_code != _START_CODE_GOP &&
        start_code != _START_CODE_SEQ) {
      LOG(ERROR) << "invalid start code in gop structure";
      return false;
    }

    // Validate that we have all the pictures after reordering
    int seq = -1;
    for (const auto& [picture_seq, pkts] : pictures) {
      if (int(picture_seq) != seq + 1) {
        LOG(ERROR) << "invalid gop, picture sequence error";
        return false;
      }
      seq = picture_seq;
    }
    if (seq == -1) {
      LOG(ERROR) << "invalid gop, no picture structures found";
      return false;
    }

    for (auto&& [_,pkts] : pictures)
      for (auto&& pkt : pkts)
        out.emplace_back(std::move(pkt));

    return true;
  }

  static inline bool _detail::_parse_picture(
      mpegts::ElementaryStream& mpeg2v_stream, std::vector<dtvcc::Packet>& out, uint16_t& picture_seq)
  {
    auto& iter = mpeg2v_stream.Iter;

    // Parse picture header (ISO 13818-2 6.2.3 / 6.3.9)
    {
      if (_next_start_code(iter, true).value_or(0xFF) != _START_CODE_PIC) {
        LOG(ERROR) << "invalid picture header start code";
        return false;
      }
      if (iter.size() < 5) { // NB: One extra byte for extra_bit nonsense
        LOG(ERROR) << "invalid picture header";
        return false;
      }
                    picture_seq  = (iter[0] << 2) | ((iter[1] >> 6) & 0b11);
      const uint8_t coding_type  = (iter[1] >> 3) & 0b111;
      const uint16_t vbv_delay   = ((iter[1] & 0b111) << 13) | (iter[2] << 5) | ((iter[3] >> 3) & 0b11111);
      if (coding_type < 1u || coding_type > 3u ||
          vbv_delay != 0xFFFF) {
        LOG(ERROR) << "invalid picture header";
        return false;
      }
      iter.remove_prefix(3); // NB: The three lowest bits of iter[3] are still unprocessed

      uint8_t extra_bit;
      if (coding_type == 1u) {
        extra_bit = (iter[0] >> 2) & 0b1; // Byte 0 Bit 5
        iter.remove_prefix(1);
      } else if (coding_type == 2u) {
        extra_bit = (iter[1] >> 6) & 0b1; // Byte 1 Bit 1
        iter.remove_prefix(2); // full_pel_forward_vector, forward_f_code
      } else { // 3u
        extra_bit = (iter[1] >> 2) & 0b1; // Byte 1 Bit 5
        iter.remove_prefix(2); // ", ", full_pel_backward_vector, backward_f_code
      }
      if (extra_bit) {
        LOG(ERROR) << "invalid picture header";
        return false;
      }
    }

    // Parse picture coding extension (ISO 13818-2 6.2.3.1 / 6.3.10)
    // NB: We skip many fields here, none of which seem important to us
    {
      if (_next_start_code(iter, true).value_or(0xFF) != _START_CODE_EXT) {
        LOG(ERROR) << "invalid picture coding extension start code";
        return false;
      }
      if (iter.size() < 5) {
        LOG(ERROR) << "invalid picture coding extension";
        return false;
      }
      const uint8_t ext_id                  = (iter[0] >> 4) & 0b1111;
      const uint8_t composite_display_flag  = (iter[4] >> 6) & 0b1;
      if (ext_id != 0b1000) {
        LOG(ERROR) << "invalid picture coding extension";
        return false;
      }
      iter.remove_prefix(5);

      if (composite_display_flag) {
        if (iter.size() < 2) {
          LOG(ERROR) << "invalid picture coding extension";
          return false;
        }
        iter.remove_prefix(2);
      }
    }

    // NB: Technically user data and extensions are allowed to come in
    // basically any order, however I have only observed EXT, USR, SLICE.

    // Parse various optional picture extensions
    while (_next_start_code(iter, false) == _START_CODE_EXT)
    {
      iter.remove_prefix(4);
      if (iter.size() < 1) {
        LOG(ERROR) << "invalid picture misc extension";
        return false;
      }
      const uint8_t ext_id = (iter[0] >> 4) & 0b1111;
      if (ext_id != 0b0011 && // Quant Matrix Extension
          ext_id != 0b0111 && // Picture Display Extension
          ext_id != 0b1010 && // Picture Temporal Scalable Extension
          ext_id != 0b1001 && // Picture Spatial Scalable Extension
          ext_id != 0b0100) { // Copyright Extension
        LOG(ERROR) << "invalid picture misc extension";
        return false;
      }
      _skipto_start_code(iter);
    }

    // Parse user data, this is where captions are found!
    while (_next_start_code(iter, false) == _START_CODE_USR)
      if (not _parse_user(mpeg2v_stream, out))
        return false;

    // Parse array of slice structures
    {
      bool found_slice = false;
      std::optional<uint8_t> start_code;
      while ((start_code = _next_start_code(iter, false))
              >= _START_CODE_SLL && start_code <= _START_CODE_SLH) {
        if (not _parse_slice(mpeg2v_stream))
          return false;
        found_slice = true;
      }
      if (not iter.empty() &&
          start_code != _START_CODE_PIC &&
          start_code != _START_CODE_GOP &&
          start_code != _START_CODE_SEQ) {
        LOG(ERROR) << "invalid start code in picture structure";
        return false;
      }
      if (not found_slice) {
        LOG(ERROR) << "invalid picture, no slices found";
        return false;
      }
    }

    return true;
  }

  static inline bool _detail::_parse_slice(mpegts::ElementaryStream& mpeg2v_stream)
  {
    auto& iter = mpeg2v_stream.Iter;

    // Parse slice header (ISO 13818-2 6.2.4 / 6.3.16)
    const auto code = _next_start_code(iter, true);
    if (not (code.has_value() && code.value() >= _START_CODE_SLL && code <= _START_CODE_SLH)) {
      LOG(ERROR) << "invalid slice header start code";
      return false;
    }

    // TODO: For now we are just skipping over all the remaining slice header
    // fields and macroblock array. It's really quite complicated and not
    // relevant to closed captions.
    _skipto_start_code(iter);
    return true;
  }

  static inline bool _detail::_parse_user(mpegts::ElementaryStream& mpeg2v_stream, std::vector<dtvcc::Packet>& out)
  {
    auto& iter = mpeg2v_stream.Iter;

    // Parse user data header (ISO 13818-2 6.2.2.2.2 / 6.3.4.1)
    // NB: ISO 13818-2 says nothing about the contents of user_data, other than
    // the fact that it's not supposed to have 23 or more consecutive 0 bits.
    if (_next_start_code(iter, true).value_or(0xFF) != _START_CODE_USR) {
      LOG(ERROR) << "invalid user header start code";
      return false;
    }

    // Parse user data identifier (ATSC A/53-4 Table 6.7)
    if (iter.size() < 4) {
      LOG(ERROR) << "invalid user data identifier";
      return false;
    }
    const uint32_t atsc_ident =
      (iter[0] << 24) |
      (iter[1] << 16) |
      (iter[2] << 8)  |
      (iter[3] << 0);
    iter.remove_prefix(4);

    // Active format description data (describes an "area of interest" inside
    // the source frame, like a letterbox). We don't care about this.
    if (atsc_ident == htobe32(*(uint32_t*)"DTG1"))
    {
      // Parse AFD data (ATSC A/53-4 Table 6.13)
      if (iter.size() < 1) {
        LOG(ERROR) << "invalid user AFD data";
        return false;
      }
      const uint8_t zero               = (iter[0] >> 7) & 0b1;
      const uint8_t has_active_format  = (iter[0] >> 6) & 0b1;
      const uint8_t reserved0          = iter[0] & 0b111111;
      if (zero ||
          reserved0 != 1u) {
        LOG(ERROR) << "invalid user AFD data";
        return false;
      }
      iter.remove_prefix(1);

      if (has_active_format) {
        if (iter.size() < 1) {
          LOG(ERROR) << "invalid user AFD data";
          return false;
        }
        const uint8_t reserved1      = (iter[0] >> 4) & 0b1111;
        const uint8_t active_format  = iter[0] & 0b1111;
        if (reserved1 != 0xF) {
          LOG(ERROR) << "invalid user AFD data";
          return false;
        }
        UNUSED(active_format);
        iter.remove_prefix(1);
      }

      if (iter.size() >= 1 && iter[0] == 0xFF)
        iter.remove_prefix(1);
    }

    // ATSC user data (could be anything defined in ATSC A/53-4 Table 6.9 but
    // we only expect / support captions).
    else if (atsc_ident == htobe32(*(uint32_t*)"GA94"))
    {
      // Parse ATSC user data (ATSC A/53-4 Table 6.8)
      if (iter.size() < 1) {
        LOG(ERROR) << "invalid user ATSC data";
        return false;
      }
      const uint8_t user_data_type_code = iter[0];
      if (user_data_type_code != 3u) { // MPEG_cc_data
        LOG(ERROR) << "invalid user ATSC type code=" << int(user_data_type_code);
        return false;
      }
      iter.remove_prefix(1);

      // Parse MPEG_cc_data (ATSC A/53-4 Table 6.10)
      // NB: This structure is all of the bytes of cc_data() (CTA-708 data)
      // followed by a all-one-bit marker.
      if (not dtvcc::parse_dtvcc_user_data(iter, out))
        return false;
      if (iter.size() < 1 || iter[0] != 0xFF) {
        LOG(ERROR) << "invalid user ATSC marker";
        return false;
      }
      iter.remove_prefix(1);
    }

    // Unknown user data, nothing we can do but skip ahead to the next start code
    else {
      LOG(WARN) << "unknown user data identifier=" << atsc_ident;
      _skipto_start_code(iter);
    }

    return true;
  }

};
