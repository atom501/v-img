#pragma once

#include <fmt/core.h>
#include <geometry/mesh.h>
#include <miniz.h>

#include <algorithm>
#include <filesystem>
#include <fstream>
#include <string>

// flags used by mitsuba serialized file format
enum class TriMeshFlags : uint32_t {
  HasNormals = 0x0001,
  HasTexcoords = 0x0002,
  HasTangents = 0x0004,  // unused
  HasColors = 0x0008,
  FaceNormals = 0x0010,
  SinglePrecision = 0x1000,
  DoublePrecision = 0x2000
};

class ZStream {
private:
  std::fstream &fs;
  size_t fsize;
  z_stream m_inflateStream;

  std::vector<std::byte> m_inflateBuffer;

public:
  ZStream(std::fstream &in_fs) : fs(in_fs) {
    // get file size
    std::streampos pos = fs.tellg();
    fs.seekg(0, fs.end);
    fsize = (size_t)fs.tellg();
    fs.seekg(pos, fs.beg);

    // init vector of bytes to use as a buffer
    m_inflateBuffer = std::vector<std::byte>(32768);

    // init decompression stream
    m_inflateStream.zalloc = Z_NULL;
    m_inflateStream.zfree = Z_NULL;
    m_inflateStream.opaque = Z_NULL;
    m_inflateStream.avail_in = 0;
    m_inflateStream.next_in = Z_NULL;

    int retval = inflateInit(&m_inflateStream);
    if (retval != Z_OK) {
      fmt::println("Could not initialize ZLIB");
    }
  }

  virtual ~ZStream() = default;

  // decompress and read size number of bytes to ptr
  void read(void *ptr, size_t size) {
    unsigned char *targetPtr = (unsigned char *)ptr;
    while (size > 0) {
      // if ran out of data to read more into buffer and reset zstream pointer and data size
      if (m_inflateStream.avail_in == 0) {
        size_t remaining = fsize - fs.tellg();
        m_inflateStream.next_in = reinterpret_cast<unsigned char *>(m_inflateBuffer.data());
        m_inflateStream.avail_in = (uInt)std::min(remaining, m_inflateBuffer.size());
        if (m_inflateStream.avail_in == 0) {
          fmt::println("Read less data than expected");
        }

        fs.read(reinterpret_cast<char *>(m_inflateBuffer.data()), m_inflateStream.avail_in);
      }

      m_inflateStream.avail_out = (uInt)size;
      m_inflateStream.next_out = targetPtr;

      // read needed bytes from zstream
      int retval = inflate(&m_inflateStream, Z_NO_FLUSH);
      switch (retval) {
        case Z_STREAM_ERROR: {
          fmt::println("inflate(): stream error!");
        }
        case Z_NEED_DICT: {
          fmt::println("inflate(): need dictionary!");
        }
        case Z_DATA_ERROR: {
          fmt::println("inflate(): data error!");
        }
        case Z_MEM_ERROR: {
          fmt::println("inflate(): memory error!");
        }
      };

      size_t outputSize = size - (size_t)m_inflateStream.avail_out;
      targetPtr += outputSize;
      size -= outputSize;

      if (size > 0 && retval == Z_STREAM_END) {
        fmt::println("inflate(): attempting to read past the end of the stream!");
      }
    }
  }
};

/*
 * Takes the path of the serialized file. The path is relative to mitsuba scene file.
 * Reads and returns a single mesh. Right now does not supports multiple meshes
 */
Mesh read_serialized_file(const std::filesystem::path &filepath, int shape_index,
                          glm::mat4 to_world);