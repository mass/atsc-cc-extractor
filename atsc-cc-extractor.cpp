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
/// - https://en.wikipedia.org/wiki/CEA-708
/// - ISO/IEC 13818-1

#include "mpegts.hpp"
#include "mpegv.hpp"
#include "utils.hpp"

#include <m/m.hpp>
#include <m/log.hpp>

int main(int argc, char** argv)
{
  // Parse arguments
  if (argc != 2) {
    LOG(ERROR) << "invalid arguments";
    return 1;
  }
  const std::string_view input_fname = argv[1];
  LOG(INFO) << "reading filename=(" << input_fname << ")";

  // Read input file data into memory
  m::byte_vec input_data;
  if (not utils::read_file(input_fname, input_data))
    return 1;
  LOG(INFO) << "read input file into memory bytes=" << input_data.size();

  // Scan through input data and parse as transport stream format
  mpegts::Pid_t pcr_pid;
  mpegts::StreamMap_t streams;
  if (not mpegts::demux(m::view(input_data), streams, pcr_pid))
    return 1;
  input_data.clear();
  input_data.shrink_to_fit();
  for (const auto& [pid, stream] : streams)
    LOG(INFO) << "found stream pid=" << pid << " num_packets=" << stream.Packets.size();

  // Ignore the SDT (service description table)
  // TODO: Investigate if there is something useful here
  streams.erase(mpegts::PID_SDT);
  LOG(INFO) << "ignoring SDT (pid=" << mpegts::PID_SDT << ")";

  // Scan the PAT (program association table) to find the PMT PID
  mpegts::Pid_t pmt_pid;
  if (streams.count(mpegts::PID_PAT) == 0) {
    LOG(ERROR) << "transport stream contained no PAT (pid=" << mpegts::PID_PAT << ")";
    return 1;
  }
  if (not mpegts::parse_pat(streams[mpegts::PID_PAT], pmt_pid))
    return 1;
  streams.erase(mpegts::PID_PAT);
  LOG(INFO) << "finished scanning PATs (pid=" << mpegts::PID_PAT << "), found PMT (pid=" << pmt_pid << ")";

  // Scan the PMT (program map table) to tag all the streams
  if (streams.count(pmt_pid) == 0) {
    LOG(ERROR) << "transport stream contained no PMT";
    return 1;
  }
  if (not mpegts::parse_pmt(streams[pmt_pid], streams, pcr_pid))
    return 1;
  streams.erase(pmt_pid);

  // Print stream type debugging information and find singular MPEG2 video stream for cc extraction
  static constexpr uint8_t STREAM_TYPE_VIDEO_MPEG2 = 2;
  static constexpr uint8_t STREAM_TYPE_AUDIO_AC3   = 129;
  for (auto it = streams.begin(); it != streams.end(); ) {
    const auto& pid = it->first;
    const auto& stream = it->second;
    if (stream.Type == 0) {
      LOG(ERROR) << "unidentified stream pid=" << pid;
      return 1;
    } else if (stream.Type == STREAM_TYPE_VIDEO_MPEG2) {
      LOG(INFO) << "identified stream pid=" << pid << " type=Video_MPEG2";
      ++it;
    } else if (stream.Type == STREAM_TYPE_AUDIO_AC3) {
      LOG(INFO) << "identified stream pid=" << pid << " type=Audio_AC3 lang=(" << stream.Lang << ")";
      it = streams.erase(it);
    } else {
      LOG(ERROR) << "unknown stream type pid=" << pid << " type=" << int(stream.Type);
      return 1;
    }
  }
  if (streams.size() != 1) {
    LOG(ERROR) << "did not find singular MPEG2 video stream";
    return 1;
  }

  // Extract the video elementary stream into a contiguous region of memory
  mpegts::ElementaryStream mpeg2v_stream;
  if (not mpegts::depacketize_stream(streams.cbegin()->second, mpeg2v_stream))
    return 1;
  streams.clear();
  LOG(INFO) << "depacketized mpeg2 video stream bytes=" << mpeg2v_stream.Data.get_read_left();

  // Extract DTVCC transport stream (CEA-708 closed captions) from MPEG2 video stream
  std::vector<mpegv::DtvccPacket> dtvcc_packets;
  if (not mpegv::extract_dtvcc_packets(mpeg2v_stream, dtvcc_packets))
    return 1;
  mpeg2v_stream.Data.free();
  LOG(INFO) << "extracted dtvcc transport stream num_packets=" << dtvcc_packets.size();

  // Extract each DTVCC "service" as a contiguous stream
  std::map<uint8_t, mpegts::ElementaryStream> dtvcc_services;
  if (not mpegv::depacketize_dtvcc_stream(dtvcc_packets, dtvcc_services))
    return 1;
  dtvcc_packets.clear();
  dtvcc_packets.shrink_to_fit();
  LOG(INFO) << "depacketized dtvcc services count=" << dtvcc_services.size();
  for (const auto& [svc_id, svc] : dtvcc_services)
    LOG(INFO) << "    "
              << " service_id=" << int(svc_id)
              << " bytes=" << svc.Data.get_read_left()
              << " packets=" << svc.Times.size();
  //TODO: More intelligent selection?
  auto svc = std::max_element(dtvcc_services.begin(), dtvcc_services.end(),
      [] (const auto& a, const auto& b) { return a.second.Data.get_read_left() < b.second.Data.get_read_left(); });
  mpegts::ElementaryStream dtvcc_stream = std::move(svc->second);
  LOG(INFO) << "selected dtvcc service id=" << int(svc->first);
  dtvcc_services.clear();

  // TODO
  if (not mpegv::process_dtvcc_stream(dtvcc_stream))
    return 1;

  //TODO: Further steps

  LOG(INFO) << "finished processing filename=(" << input_fname << ")";
  return 0;
}
