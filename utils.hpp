#pragma once

#include <m/m.hpp>

#include <cstring>
#include <iostream>
#include <string_view>
#include <utility>
#include <vector>

////////////////////////////////////////////////////////////////////////////////
/// Various useful utilities and common types
namespace utils
{
  // Read the entire contents of the given binary file into a byte vector
  bool read_file(std::string_view filename, m::byte_vec& out);

  /// Implementation ///////////////////////////////////////////////////////////

  inline bool read_file(std::string_view filename, m::byte_vec& out)
  {
    FILE* fp = nullptr;
    if ((fp = ::fopen(filename.data(), "r")) == nullptr) {
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
};
