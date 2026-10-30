#include "r3v_ros/voxel_stream.hpp"

#include <zstd.h>

#include <cstring>

namespace r3v_ros {

namespace {

using msg::VoxelMapUpdate;

// Bigger payloads are taken for corrupt rather than allocated.
constexpr size_t kMaxPayload = size_t{ 1 } << 30;

}  // namespace

bool validCompressionLevel(int level) {
  return level >= ZSTD_minCLevel() && level <= ZSTD_maxCLevel();
}

std::vector<uint8_t> encodePayload(
  const void* data, size_t bytes, uint8_t compression, int level) {
  const auto* begin = static_cast<const uint8_t*>(data);
  if (compression == VoxelMapUpdate::COMPRESSION_NONE) {
    return std::vector<uint8_t>(begin, begin + bytes);
  }
  if (compression != VoxelMapUpdate::COMPRESSION_ZSTD) {
    throw std::runtime_error("unknown compression " + std::to_string(compression));
  }
  // Writes the decompressed size into the frame, decodedSize() reads it.
  std::vector<uint8_t> out(ZSTD_compressBound(bytes));
  const size_t size = ZSTD_compress(out.data(), out.size(), data, bytes, level);
  if (ZSTD_isError(size)) {
    throw std::runtime_error(std::string("zstd compression failed: ") + ZSTD_getErrorName(size));
  }
  out.resize(size);
  return out;
}

size_t decodedSize(const VoxelMapUpdate& msg) {
  if (msg.compression == VoxelMapUpdate::COMPRESSION_NONE) {
    return msg.data.size();
  }
  if (msg.compression != VoxelMapUpdate::COMPRESSION_ZSTD) {
    throw std::runtime_error("unknown compression " + std::to_string(msg.compression));
  }
  const unsigned long long size =  // NOLINT(runtime/int): zstd's type
    ZSTD_getFrameContentSize(msg.data.data(), msg.data.size());
  if (size == ZSTD_CONTENTSIZE_ERROR) {
    throw std::runtime_error("payload is not a zstd frame");
  }
  if (size == ZSTD_CONTENTSIZE_UNKNOWN) {
    throw std::runtime_error("zstd frame does not tell its decompressed size");
  }
  if (size > kMaxPayload) {
    throw std::runtime_error("zstd frame decompresses to " + std::to_string(size) + " bytes");
  }
  return static_cast<size_t>(size);
}

void decodeInto(const VoxelMapUpdate& msg, void* out, size_t bytes) {
  if (msg.compression == VoxelMapUpdate::COMPRESSION_NONE) {
    if (bytes != msg.data.size()) {
      throw std::runtime_error("payload size changed");
    }
    if (bytes > 0) {
      std::memcpy(out, msg.data.data(), bytes);
    }
    return;
  }
  const size_t size = ZSTD_decompress(out, bytes, msg.data.data(), msg.data.size());
  if (ZSTD_isError(size)) {
    throw std::runtime_error(std::string("zstd decompression failed: ") + ZSTD_getErrorName(size));
  }
  if (size != bytes) {
    throw std::runtime_error(
            "zstd frame decompressed to " + std::to_string(size) + " bytes instead of " +
            std::to_string(bytes));
  }
}

}  // namespace r3v_ros
