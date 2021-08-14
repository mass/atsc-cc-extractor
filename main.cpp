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
#include "mpegv.hpp"

int main(int argc, char** argv)
{
  // Parse arguments
  if (argc != 2) {
    std::cerr << "invalid arguments" << std::endl;
    return 1;
  }
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
  {
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
  }

  // Extract the elementary stream into a contiguous region of memory
  utils::StreamBuffer mpeg2v_stream;
  if (!mpegts::depacketize_stream(streams.cbegin()->second.Packets, mpeg2v_stream))
    return 1;
  streams.clear();
  std::cout << "depacketized mpeg2 video stream bytes=" << mpeg2v_stream.getReadLeft() << std::endl;

  // Extract DTVCC transport stream (CEA-708 closed captions) from MPEG2 video stream
  std::vector<utils::StreamBuffer> dtvcc_packets;
  if (!mpegv::extract_dtvcc_packets(mpeg2v_stream, dtvcc_packets))
    return 1;
  mpeg2v_stream.clear();
  std::cout << "extracted dtvcc transport stream num_packets=" << dtvcc_packets.size() << std::endl;

  return 0;
}
