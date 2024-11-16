#pragma once

#include <u/u.hpp>
#include <u/log.hpp>

#include <cstdio>
#include <cstring>
#include <string_view>

////////////////////////////////////////////////////////////////////////////////
/// Various useful utilities and common types
namespace utils
{
  // Read the entire contents of the given binary file into a byte vector
  static inline bool read_file(std::string_view filename, u::byte_vec& out);

  /// Implementation ///////////////////////////////////////////////////////////

  static inline bool read_file(std::string_view filename, u::byte_vec& out)
  {
    FILE* fp = nullptr;
    if ((fp = ::fopen(filename.data(), "r")) == nullptr) {
      LOG(ERROR) << "error opening file error=(" << std::strerror(errno) << ")";
      return false;
    }

    ssize_t file_size = 0;
    if (::fseek(fp, 0, SEEK_END) != 0 || (file_size = ::ftell(fp)) < 0 || ::fseek(fp, 0, SEEK_SET) != 0) {
      LOG(ERROR) << "error finding file size error=(" << std::strerror(errno) << ")";
      ::fclose(fp);
      return false;
    }

    out.resize(file_size);
    if (::fread(out.data(), 1, file_size, fp) != size_t(file_size)) {
      LOG(ERROR) << "error reading file error=(" << std::strerror(errno) << ")";
      out.clear();
      out.shrink_to_fit();
      ::fclose(fp);
      return false;
    }

    ::fclose(fp);

    return true;
  }
};
