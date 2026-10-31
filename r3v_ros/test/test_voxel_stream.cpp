#include <gtest/gtest.h>

#include <cstdint>
#include <cstring>
#include <map>
#include <random>
#include <stdexcept>
#include <tuple>
#include <vector>

#include "r3v_ros/voxel_map.hpp"
#include "r3v_ros/voxel_stream.hpp"

namespace r3v_ros {
namespace {

using msg::VoxelMapUpdate;
using Blocks = std::map<std::tuple<int32_t, int32_t, int32_t>, uint32_t>;

VoxelMapUpdate encode(const void* data, size_t bytes, uint8_t compression) {
  VoxelMapUpdate msg;
  msg.compression = compression;
  msg.data = encodePayload(data, bytes, compression, 1);
  return msg;
}

// Non-empty blocks of `groups`, written independently of VoxelMap, after
// group_blocks in r3v-renderer's geometry.rs.
Blocks blocksOf(const std::vector<WireGroup>& groups) {
  Blocks blocks;
  for (const WireGroup& group : groups) {
    for (size_t index = 0; index < kGroupPages; ++index) {
      const uint64_t word = group.pages[index];
      if (word == 0) {
        continue;
      }
      int32_t offset[3] = { 0, 0, 0 };
      for (size_t bit = 0; bit < 12; ++bit) {
        offset[bit % 3] |= static_cast<int32_t>((index >> bit) & 1u) << (bit / 3);
      }
      const int32_t x = group.key.x * 64 + offset[0] * 4;
      const int32_t y = group.key.y * 64 + offset[1] * 4;
      const int32_t z = group.key.z * 64 + offset[2] * 4;
      const auto low = static_cast<uint32_t>(word);
      const auto high = static_cast<uint32_t>(word >> 32);
      if (low != 0) {
        blocks[{ x, y, z }] = low;
      }
      if (high != 0) {
        blocks[{ x, y, z + 2 }] = high;
      }
    }
  }
  return blocks;
}

// Blocks with negative and positive coordinates, in both halves of their
// pages, several at the same origin.
std::vector<WireBlock> randomBlocks(std::mt19937& rng, size_t count) {
  std::uniform_int_distribution<int32_t> coord(-40, 40);
  std::uniform_int_distribution<uint32_t> mask(0, 3);
  std::vector<WireBlock> blocks;
  for (size_t i = 0; i < count; ++i) {
    // Mostly set, sometimes emptied.
    const uint32_t m = mask(rng) == 0 ? 0 : static_cast<uint32_t>(rng()) | 1u;
    blocks.push_back({ coord(rng) * 4, coord(rng) * 4, coord(rng) * 2, m });
  }
  return blocks;
}

Blocks lastWins(const std::vector<WireBlock>& blocks, Blocks into = { }) {
  for (const WireBlock& block : blocks) {
    if (block.mask == 0) {
      into.erase({ block.x, block.y, block.z });
    } else {
      into[{ block.x, block.y, block.z }] = block.mask;
    }
  }
  return into;
}

TEST(VoxelStream, RoundTripsBlocks) {
  const std::vector<WireBlock> blocks = { { 0, 4, -2, 7 }, { -64, 128, 6, 0xffffffffu } };
  for (const uint8_t compression :
    { VoxelMapUpdate::COMPRESSION_NONE, VoxelMapUpdate::COMPRESSION_ZSTD }) {
    const auto decoded = decodePayload<WireBlock>(
      encode(blocks.data(), blocks.size() * sizeof(WireBlock), compression));
    ASSERT_EQ(decoded.size(), blocks.size());
    EXPECT_EQ(std::memcmp(decoded.data(), blocks.data(), blocks.size() * sizeof(WireBlock)), 0);
  }
}

TEST(VoxelStream, CompressesSparseGroups) {
  std::vector<WireGroup> groups(2);
  groups[0].key = { -1, 0, 3, 0 };
  groups[1].key = { 5, -7, 0, 0 };
  for (size_t page = 0; page < kGroupPages; page += 16) {
    groups[0].pages[page] = 0x0123456789abcdefull ^ page;
  }
  groups[1].pages[kGroupPages - 1] = 1;
  const VoxelMapUpdate msg = encode(
    groups.data(), groups.size() * sizeof(WireGroup), VoxelMapUpdate::COMPRESSION_ZSTD);
  EXPECT_LT(msg.data.size(), groups.size() * sizeof(WireGroup) / 8);
  const auto decoded = decodePayload<WireGroup>(msg);
  ASSERT_EQ(decoded.size(), groups.size());
  EXPECT_EQ(std::memcmp(decoded.data(), groups.data(), groups.size() * sizeof(WireGroup)), 0);
}

TEST(VoxelStream, RoundTripsEmptyPayloads) {
  for (const uint8_t compression :
    { VoxelMapUpdate::COMPRESSION_NONE, VoxelMapUpdate::COMPRESSION_ZSTD }) {
    EXPECT_TRUE(decodePayload<WireGroup>(encode(nullptr, 0, compression)).empty());
  }
}

TEST(VoxelStream, RejectsBadPayloads) {
  const std::vector<WireBlock> blocks(10, WireBlock{ 4, 4, 2, 1 });
  const size_t bytes = blocks.size() * sizeof(WireBlock);

  VoxelMapUpdate partial = encode(blocks.data(), bytes - 1, VoxelMapUpdate::COMPRESSION_NONE);
  EXPECT_THROW(decodePayload<WireBlock>(partial), std::runtime_error);
  partial = encode(blocks.data(), bytes - 1, VoxelMapUpdate::COMPRESSION_ZSTD);
  EXPECT_THROW(decodePayload<WireBlock>(partial), std::runtime_error);

  VoxelMapUpdate truncated = encode(blocks.data(), bytes, VoxelMapUpdate::COMPRESSION_ZSTD);
  truncated.data.resize(truncated.data.size() - 2);
  EXPECT_THROW(decodePayload<WireBlock>(truncated), std::runtime_error);

  VoxelMapUpdate garbage;
  garbage.compression = VoxelMapUpdate::COMPRESSION_ZSTD;
  garbage.data.assign(64, 0xab);
  EXPECT_THROW(decodePayload<WireBlock>(garbage), std::runtime_error);

  VoxelMapUpdate unknown = encode(blocks.data(), bytes, VoxelMapUpdate::COMPRESSION_NONE);
  unknown.compression = 7;
  EXPECT_THROW(decodePayload<WireBlock>(unknown), std::runtime_error);
  EXPECT_THROW(encodePayload(blocks.data(), bytes, 7, 1), std::runtime_error);
}

TEST(VoxelMap, ValidatesBlocksAndKeys) {
  EXPECT_TRUE(validBlock({ -4, 8, -2, 1 }));
  EXPECT_FALSE(validBlock({ -2, 8, 0, 1 }));
  EXPECT_FALSE(validBlock({ 0, 1, 0, 1 }));
  EXPECT_FALSE(validBlock({ 0, 0, 3, 1 }));
  EXPECT_TRUE(validGroupKey({ -(1 << 25), (1 << 25) - 1, 0, 0 }));
  EXPECT_FALSE(validGroupKey({ 1 << 25, 0, 0, 0 }));
  EXPECT_FALSE(validGroupKey({ 0, 0, -(1 << 25) - 1, 0 }));
}

TEST(VoxelMap, AppliesBlocks) {
  std::mt19937 rng(42);
  VoxelMap map;
  Blocks expected;
  for (int round = 0; round < 5; ++round) {
    const std::vector<WireBlock> blocks = randomBlocks(rng, 2000);
    map.apply(blocks);
    expected = lastWins(blocks, expected);
    EXPECT_EQ(blocksOf(map.groups()), expected);
  }
}

TEST(VoxelMap, DropsEmptiedGroups) {
  VoxelMap map;
  map.apply({ { -4, 0, -2, 5 }, { 60, 60, 62, 3 }, { 64, 0, 0, 1 } });
  EXPECT_EQ(map.groups().size(), 3u);
  map.apply({ { -4, 0, -2, 0 } });
  ASSERT_EQ(map.groups().size(), 2u);
  map.apply({ { 60, 60, 62, 0 } });
  ASSERT_EQ(map.groups().size(), 1u);
  EXPECT_EQ(map.groups()[0].key.x, 1);
  map.apply({ { 64, 0, 0, 0 }, { 128, 0, 0, 0 } });
  EXPECT_TRUE(map.groups().empty());
}

TEST(VoxelMap, ReplacesAndAppliesOnTop) {
  std::mt19937 rng(7);
  VoxelMap source;
  const std::vector<WireBlock> first = randomBlocks(rng, 3000);
  source.apply(first);

  VoxelMap map;
  map.apply(randomBlocks(rng, 500));  // replaced
  map.replace(source.groups());
  EXPECT_EQ(blocksOf(map.groups()), lastWins(first));

  const std::vector<WireBlock> second = randomBlocks(rng, 3000);
  map.apply(second);
  EXPECT_EQ(blocksOf(map.groups()), lastWins(second, lastWins(first)));
}

TEST(VoxelMap, MergesGroupsSharingAKey) {
  std::vector<WireGroup> groups(3);
  groups[0].key = { 2, -3, 0, 0 };
  groups[0].pages[0] = 0x1;
  groups[1].key = { 2, -3, 0, 0 };
  groups[1].pages[0] = 0x100000000ull;
  groups[1].pages[5] = 0x2;  // x and z offset 1 page
  groups[2].key = { 0, 0, 0, 0 };  // empty, dropped
  VoxelMap map;
  map.replace(groups);
  EXPECT_EQ(
    blocksOf(map.groups()),
    (Blocks{ { { 128, -192, 0 }, 1 }, { { 128, -192, 2 }, 1 }, { { 132, -192, 4 }, 2 } }));
}

}  // namespace
}  // namespace r3v_ros
