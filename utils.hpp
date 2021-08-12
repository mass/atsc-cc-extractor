#pragma once

#include <cstring>
#include <iostream>
#include <string_view>
#include <utility>
#include <vector>

////////////////////////////////////////////////////////////////////////////////
/// Various useful utilities and common types
namespace utils
{
  // Helpers for unsigned char (byte) values
  using byte_view = std::basic_string_view<uint8_t>;
  using byte_vec  = std::vector<uint8_t>;
  inline byte_view byte_vec_to_view(const byte_vec& vec) { return byte_view(vec.data(), vec.size()); }

  // Read the entire contents of the given binary file into a byte vector
  bool read_file(std::string_view filename, byte_vec& out);

  // Contiguous memory buffer with read/write position tracking
  class StreamBuffer
  {
    public:

      StreamBuffer();
      ~StreamBuffer();
      StreamBuffer(StreamBuffer&& o);

      void clear();

      inline const uint8_t* getRead() const { return _read; }
      inline size_t getReadLeft() const { return _write - _read; }
      inline byte_view getReadView() const { return byte_view(_read, _write - _read); }
      void advanceRead(size_t n);

      inline uint8_t* getWrite() { return _write; }
      inline size_t getWriteLeft() const { return (_buffer + _size) - _write; }
      inline void advanceWrite(size_t n) { _write += n; }
      void reserveWrite(size_t n);
      inline void write(byte_view data) { write(data.data(), data.size()); }
      inline void write(const void* data, size_t n) {
        reserveWrite(n);
        ::memcpy(getWrite(), data, n);
        advanceWrite(n);
      }

    private:

      StreamBuffer(const StreamBuffer& o) = delete;
      StreamBuffer& operator=(const StreamBuffer& o) = delete;
      StreamBuffer& operator=(StreamBuffer&& o) = delete;

      uint8_t* _buffer = nullptr;
      size_t _size = 0;
      uint8_t* _read = nullptr;
      uint8_t* _write = nullptr;
  };

  /// Implementation ///////////////////////////////////////////////////////////

  inline bool read_file(std::string_view filename, byte_vec& out)
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

  inline StreamBuffer::StreamBuffer()
  {
    _size = 256;
    _buffer = (uint8_t*) ::malloc(_size);
    _read = _buffer;
    _write = _buffer;
  }

  inline StreamBuffer::~StreamBuffer()
  {
    if (_buffer != nullptr)
      ::free(_buffer);
    _buffer = nullptr;
    _size = 0;
    _read = nullptr;
    _write = nullptr;
  }

  inline StreamBuffer::StreamBuffer(StreamBuffer&& o)
  {
    _buffer = std::exchange(o._buffer, nullptr);
    _size = std::exchange(o._size, 0);
    _read = std::exchange(o._read, nullptr);
    _write = std::exchange(o._write, nullptr);
  }

  inline void StreamBuffer::clear()
  {
    _read = _buffer;
    _write = _buffer;
  }

  inline void StreamBuffer::advanceRead(size_t n)
  {
    _read += std::min(n, getReadLeft());
    if (getReadLeft() <= 0) {
      _read = _buffer;
      _write = _buffer;
    }
  }

  inline void StreamBuffer::reserveWrite(size_t n)
  {
    const size_t used = getReadLeft();
    const size_t left = getWriteLeft();

    if (left >= n)
      return;

    _size = 2 * (n + used);
    uint8_t* newBuf = (uint8_t*) ::malloc(_size);
    ::memcpy(newBuf, getRead(), used);
    ::free(_buffer);
    _buffer = newBuf;
    _read = _buffer;
    _write = _buffer + used;
  }

};
