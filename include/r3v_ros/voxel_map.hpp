#pragma once

#include <cstddef>
#include <cstdint>
#include <unordered_map>
#include <vector>

#include "r3v_ros/voxel_stream.hpp"

namespace r3v_ros {

// Whether `block` lies on the 4x4x2 grid the tesselator takes (x and y
// multiples of 4, z of 2).
bool validBlock(const WireBlock& block);

// Whether the voxels of the group with key `key` fit int32_t, as the
// tesselator requires (every coordinate in [-2^25, 2^25)).
bool validGroupKey(const WireBlock& key);

// Constructed voxels of the map the voxel stream describes, as the groups of
// 64x64x64 voxels the tesselator replaces its geometry with. Groups that lose
// their last voxel are dropped.
class VoxelMap
{
public:
  // Replaces the map. Groups sharing a key add up.
  void replace(std::vector<WireGroup> groups);

  // Every block replaces the mask the map had for it, so the last of
  // several at the same origin wins. Blocks must be validBlock().
  void apply(const std::vector<WireBlock>& blocks);

  void clear();

  // Groups holding at least one voxel, with distinct keys, in no order.
  const std::vector<WireGroup>& groups() const {
    return groups_;
  }

private:
  // Sets the word `page` of group `index` and keeps used_ up to date.
  // Returns false when the group lost its last voxel.
  bool setPage(size_t index, size_t page, uint64_t word);
  void removeGroup(size_t index);

  std::vector<WireGroup> groups_;
  std::vector<size_t> used_;  // non-zero pages of every group, as groups_
  std::unordered_map<VoxelKey, size_t, VoxelKeyHash> index_;  // group key -> index
};

}  // namespace r3v_ros
