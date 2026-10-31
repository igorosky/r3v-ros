#pragma once

#include <cstddef>
#include <cstdint>
#include <stdexcept>
#include <string>
#include <vector>

#include "r3v_interfaces/msg/voxel_map_update.hpp"

namespace r3v_ros {

// The voxel stream messages (VoxelMapUpdate, FullSyncRequest).
namespace msg = r3v_interfaces::msg;

// Payload of the voxel stream (r3v_interfaces/msg/VoxelMapUpdate), shared by the
// voxelizer and the renderer node. It copies the structs of the r3v crates
// as they are, which are little endian on every platform they run on.
static_assert(
  __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__, "the voxel stream payload is little endian");

// Block of 4x4x2 voxels, the layout of r3v-voxelizer's PointI and
// r3v-renderer's R3vPointI. Voxel (x, y, z) of the block is bit
// x + 4 * y + 16 * z of `mask`.
struct WireBlock {
  int32_t x;
  int32_t y;
  int32_t z;
  uint32_t mask;
};

// Words of a WireGroup, one per page of 4x4x4 voxels.
constexpr size_t kGroupPages = 4096;

// Group of 64x64x64 voxels, the layout of r3v-voxelizer's VoxelGroup and
// r3v-renderer's R3vVoxelGroup. `key` holds the voxel coordinates shifted
// right by 6 (mask 0). `pages[k]` is the page whose offset in the group, bits
// of x, y and z interleaved from the lowest one, is `k`; voxel (x, y, z) of
// the page is bit x + 4 * y + 16 * z of it.
struct WireGroup {
  WireBlock key;
  uint64_t pages[kGroupPages];
};

// Voxel coordinates of a block origin or a group key.
struct VoxelKey {
  int32_t x;
  int32_t y;
  int32_t z;
  bool operator==(const VoxelKey& other) const {
    return x == other.x && y == other.y && z == other.z;
  }
};
struct VoxelKeyHash {
  size_t operator()(const VoxelKey& key) const {
    return (static_cast<size_t>(static_cast<uint32_t>(key.x)) * 73856093u) ^
           (static_cast<size_t>(static_cast<uint32_t>(key.y)) * 19349663u) ^
           (static_cast<size_t>(static_cast<uint32_t>(key.z)) * 83492791u);
  }
};

// Whether zstd takes `level`.
bool validCompressionLevel(int level);

// The data of a VoxelMapUpdate holding `bytes` from `data`, compressed by
// `compression` (a VoxelMapUpdate::COMPRESSION_* value) at zstd `level`.
// Throws std::runtime_error when compressing fails.
std::vector<uint8_t> encodePayload(
  const void* data, size_t bytes, uint8_t compression, int level);

// Size of the data of `msg` once decompressed. Throws std::runtime_error for
// an unknown compression or a size that is not known or too big.
size_t decodedSize(const msg::VoxelMapUpdate& msg);

// Decompresses the data of `msg` into `bytes` (its decodedSize()) at `out`.
// Throws std::runtime_error when the data is corrupt.
void decodeInto(const msg::VoxelMapUpdate& msg, void* out, size_t bytes);

// The data of `msg` as an array of `T` (WireBlock or WireGroup). Throws
// std::runtime_error when it is corrupt or not a whole number of `T`.
template<typename T>
std::vector<T> decodePayload(const msg::VoxelMapUpdate& msg) {
  const size_t bytes = decodedSize(msg);
  if (bytes % sizeof(T) != 0) {
    throw std::runtime_error(
            "payload of " + std::to_string(bytes) + " bytes is not a whole number of " +
            std::to_string(sizeof(T)) + " byte elements");
  }
  std::vector<T> out(bytes / sizeof(T));
  decodeInto(msg, out.data(), bytes);
  return out;
}

}  // namespace r3v_ros
