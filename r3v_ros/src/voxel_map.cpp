#include "r3v_ros/voxel_map.hpp"

#include <utility>

namespace r3v_ros {

namespace {

constexpr int32_t kGroupEdge = 64;  // voxels
constexpr int32_t kPageEdge = 4;  // voxels
constexpr int32_t kKeyLimit = int32_t{ 1 } << 25;

// floor(c / kGroupEdge), the group key coordinate of voxel coordinate `c`.
int32_t groupCoord(int32_t c) {
  return c >= 0 ? c / kGroupEdge : -((-(c + 1)) / kGroupEdge) - 1;
}

// Index of the page at (x, y, z) pages (0..15 each) from the group origin:
// bits of x, y and z interleaved from the lowest one.
size_t pageIndex(uint32_t x, uint32_t y, uint32_t z) {
  size_t index = 0;
  for (uint32_t bit = 0; bit < 4; ++bit) {
    index |= static_cast<size_t>((x >> bit) & 1u) << (3 * bit);
    index |= static_cast<size_t>((y >> bit) & 1u) << (3 * bit + 1);
    index |= static_cast<size_t>((z >> bit) & 1u) << (3 * bit + 2);
  }
  return index;
}

size_t usedPages(const WireGroup& group) {
  size_t used = 0;
  for (const uint64_t word : group.pages) {
    if (word != 0) {
      ++used;
    }
  }
  return used;
}

VoxelKey keyOf(const WireGroup& group) {
  return { group.key.x, group.key.y, group.key.z };
}

}  // namespace

bool validBlock(const WireBlock& block) {
  return block.x % kPageEdge == 0 && block.y % kPageEdge == 0 && block.z % 2 == 0;
}

bool validGroupKey(const WireBlock& key) {
  const auto valid = [](int32_t c) { return c >= -kKeyLimit && c < kKeyLimit; };
  return valid(key.x) && valid(key.y) && valid(key.z);
}

void VoxelMap::replace(std::vector<WireGroup> groups) {
  clear();
  groups_ = std::move(groups);
  // Groups sharing a key are merged into the first of them.
  size_t kept = 0;
  for (size_t i = 0; i < groups_.size(); ++i) {
    const auto [it, inserted] = index_.try_emplace(keyOf(groups_[i]), kept);
    if (inserted) {
      if (kept != i) {
        groups_[kept] = groups_[i];
      }
      ++kept;
      continue;
    }
    WireGroup& into = groups_[it->second];
    for (size_t page = 0; page < kGroupPages; ++page) {
      into.pages[page] |= groups_[i].pages[page];
    }
  }
  groups_.resize(kept);
  used_.reserve(kept);
  for (const WireGroup& group : groups_) {
    used_.push_back(usedPages(group));
  }
  // Backwards: removeGroup() moves the last group, already looked at, here.
  for (size_t i = groups_.size(); i-- > 0; ) {
    if (used_[i] == 0) {
      removeGroup(i);
    }
  }
}

void VoxelMap::apply(const std::vector<WireBlock>& blocks) {
  for (const WireBlock& block : blocks) {
    const VoxelKey key{ groupCoord(block.x), groupCoord(block.y), groupCoord(block.z) };
    auto it = index_.find(key);
    if (it == index_.end()) {
      if (block.mask == 0) {
        continue;
      }
      it = index_.emplace(key, groups_.size()).first;
      groups_.emplace_back();
      groups_.back().key = { key.x, key.y, key.z, 0 };
      used_.push_back(0);
    }
    const size_t index = it->second;
    // Voxels from the group origin, 0..63 (group * 64 fits int32_t).
    const auto offset = [](int32_t c, int32_t group) {
        return static_cast<uint32_t>(c - group * kGroupEdge);
      };
    const uint32_t x = offset(block.x, key.x);
    const uint32_t y = offset(block.y, key.y);
    const uint32_t z = offset(block.z, key.z);
    const size_t page = pageIndex(x / kPageEdge, y / kPageEdge, z / kPageEdge);
    // A block is the lower or the upper half of its page.
    const uint32_t shift = (z & 2u) != 0 ? 32 : 0;
    const uint64_t old = groups_[index].pages[page];
    const uint64_t word =
      (old & ~(uint64_t{ 0xffffffffu } << shift)) | (uint64_t{ block.mask } << shift);
    if (!setPage(index, page, word)) {
      removeGroup(index);
    }
  }
}

void VoxelMap::clear() {
  // Frees the memory too: a disabled renderer drops its map.
  std::vector<WireGroup>().swap(groups_);
  std::vector<size_t>().swap(used_);
  index_.clear();
}

bool VoxelMap::setPage(size_t index, size_t page, uint64_t word) {
  uint64_t& old = groups_[index].pages[page];
  if (old == 0 && word != 0) {
    ++used_[index];
  } else if (old != 0 && word == 0) {
    --used_[index];
  }
  old = word;
  return used_[index] != 0;
}

void VoxelMap::removeGroup(size_t index) {
  index_.erase(keyOf(groups_[index]));
  const size_t last = groups_.size() - 1;
  if (index != last) {
    groups_[index] = groups_[last];
    used_[index] = used_[last];
    index_[keyOf(groups_[index])] = index;
  }
  groups_.pop_back();
  used_.pop_back();
}

}  // namespace r3v_ros
