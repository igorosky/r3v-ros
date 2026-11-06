#include <gtest/gtest.h>

#include <cstdint>
#include <set>
#include <tuple>
#include <vector>

#include "r3v_ros/voxel_map.hpp"
#include "r3v_ros/voxel_stream.hpp"
#include "r3v_ros/voxelizer_handles.hpp"

// What the voxelizer node puts on the downsampled stream, straight from
// r3v-voxelizer: blocks and groups in downsampled voxels, which a renderer
// takes like the full map at resolution * voxel_size.

namespace r3v_ros {
namespace {

using Voxel = std::tuple<int32_t, int32_t, int32_t>;

std::vector<WireBlock> blocksOf(const rv::VoxelsUpdate& update) {
  const auto* data = reinterpret_cast<const WireBlock*>(update.data);
  return std::vector<WireBlock>(data, data + update.len);
}

std::vector<WireGroup> groupsOf(const rv::VoxelGroups& groups) {
  const auto* data = reinterpret_cast<const WireGroup*>(groups.data);
  return std::vector<WireGroup>(data, data + groups.len);
}

// Voxels of `groups`, written after the layout documented on WireGroup.
std::set<Voxel> voxelsOf(const std::vector<WireGroup>& groups) {
  std::set<Voxel> voxels;
  for (const WireGroup& group : groups) {
    for (size_t index = 0; index < kGroupPages; ++index) {
      const uint64_t word = group.pages[index];
      int32_t page[3] = { 0, 0, 0 };
      for (size_t bit = 0; bit < 12; ++bit) {
        page[bit % 3] |= static_cast<int32_t>((index >> bit) & 1u) << (bit / 3);
      }
      for (int32_t bit = 0; bit < 64; ++bit) {
        if (((word >> bit) & 1u) == 0) {
          continue;
        }
        const int32_t x = group.key.x * 64 + page[0] * 4 + (bit & 3);
        const int32_t y = group.key.y * 64 + page[1] * 4 + ((bit >> 2) & 3);
        const int32_t z = group.key.z * 64 + page[2] * 4 + (bit >> 4);
        voxels.insert({ x, y, z });
      }
    }
  }
  return voxels;
}

// Resolution 1 m; one hit of intensity 1 constructs a voxel.
VoxelizerPtr makeVoxelizer() {
  return VoxelizerPtr(rv::create_single_threaded_voxelizer(1.0f, 1.0f, 1, 2, 0, 2, 10, true));
}

// A wall of points at x = `x` m, y in [-10, 10] m and z in [-5, 5] m, seen
// from the origin.
void addWall(void* voxelizer, float x) {
  std::vector<rv::PointF> points;
  for (int y = -10; y <= 10; ++y) {
    for (int z = -5; z <= 5; ++z) {
      points.push_back(
        { x + 0.5f, static_cast<float>(y) + 0.5f, static_cast<float>(z) + 0.5f, 1.0f });
    }
  }
  const OwnedUpdate update(
    rv::add_points(voxelizer, { 0.5f, 0.5f, 0.5f, 0.0f }, points.data(), points.size()));
}

// Downsampled voxels (size 4, threshold 1) of the wall at x = `x`.
std::set<Voxel> downsampledWall(int32_t x) {
  std::set<Voxel> voxels;
  for (int32_t y = -10 >> 2; y <= 10 >> 2; ++y) {
    for (int32_t z = -5 >> 2; z <= 5 >> 2; ++z) {
      voxels.insert({ x >> 2, y, z });
    }
  }
  return voxels;
}

TEST(DownsampledStream, DeltasBuildTheWholeMap) {
  const VoxelizerPtr voxelizer = makeVoxelizer();
  ASSERT_TRUE(voxelizer);
  ASSERT_TRUE(rv::set_downsampling_params(voxelizer.get(), 4, 1));

  VoxelMap from_deltas;
  for (const float x : { 20.0f, -30.0f }) {
    addWall(voxelizer.get(), x);
    const OwnedUpdate changes(rv::get_downsampled(voxelizer.get()));
    const std::vector<WireBlock> blocks = blocksOf(*changes);
    EXPECT_FALSE(blocks.empty());
    for (const WireBlock& block : blocks) {
      EXPECT_TRUE(validBlock(block));
    }
    from_deltas.apply(blocks);
  }

  const OwnedVoxelGroups all(rv::get_all_downsampled(voxelizer.get()));
  VoxelMap whole;
  whole.replace(groupsOf(*all));
  std::set<Voxel> expected = downsampledWall(20);
  const std::set<Voxel> second = downsampledWall(-30);
  expected.insert(second.begin(), second.end());
  EXPECT_EQ(voxelsOf(whole.groups()), expected);
  EXPECT_EQ(voxelsOf(from_deltas.groups()), expected);

  // A follower that lost its copy gets every present block again.
  rv::clear_downsampled(voxelizer.get());
  const OwnedUpdate again(rv::get_downsampled(voxelizer.get()));
  VoxelMap rebuilt;
  rebuilt.apply(blocksOf(*again));
  EXPECT_EQ(voxelsOf(rebuilt.groups()), expected);
}

TEST(DownsampledStream, ClearAndDisableEmptyIt) {
  const VoxelizerPtr voxelizer = makeVoxelizer();
  ASSERT_TRUE(voxelizer);
  ASSERT_TRUE(rv::set_downsampling_params(voxelizer.get(), 4, 1));
  addWall(voxelizer.get(), 20.0f);
  {
    const OwnedUpdate changes(rv::get_downsampled(voxelizer.get()));
    EXPECT_GT(changes->len, 0u);
  }

  // clear_voxelizer keeps the downsampling settings.
  rv::clear_voxelizer(voxelizer.get());
  {
    const OwnedUpdate changes(rv::get_downsampled(voxelizer.get()));
    const OwnedVoxelGroups all(rv::get_all_downsampled(voxelizer.get()));
    const OwnedVoxelGroups full(rv::get_all_points_compressed(voxelizer.get()));
    EXPECT_EQ(changes->len, 0u);
    EXPECT_EQ(all->len, 0u);
    EXPECT_EQ(full->len, 0u);
  }
  addWall(voxelizer.get(), 20.0f);
  {
    const OwnedVoxelGroups all(rv::get_all_downsampled(voxelizer.get()));
    EXPECT_EQ(voxelsOf(groupsOf(*all)), downsampledWall(20));
  }

  rv::disable_downsampling(voxelizer.get());
  {
    const OwnedUpdate changes(rv::get_downsampled(voxelizer.get()));
    const OwnedVoxelGroups all(rv::get_all_downsampled(voxelizer.get()));
    const OwnedVoxelGroups full(rv::get_all_points_compressed(voxelizer.get()));
    EXPECT_EQ(changes->len, 0u);
    EXPECT_EQ(all->len, 0u);
    EXPECT_GT(full->len, 0u);
  }

  // What validDownsampling refuses, the voxelizer refuses too.
  EXPECT_FALSE(rv::set_downsampling_params(voxelizer.get(), 3, 1));
  EXPECT_FALSE(rv::set_downsampling_params(voxelizer.get(), 2, 9));
}

}  // namespace
}  // namespace r3v_ros
