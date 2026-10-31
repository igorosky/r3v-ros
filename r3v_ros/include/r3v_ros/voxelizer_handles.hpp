#pragma once

#include <memory>

#include "r3v-voxelizer.hpp"

namespace r3v_ros {

namespace rv = r3v::voxelizer;

struct VoxelizerDeleter {
  void operator()(void* p) const { rv::drop_voxelizer(p); }
};

using VoxelizerPtr = std::unique_ptr<void, VoxelizerDeleter>;

// Frees a result of r3v-voxelizer with `Drop` when it goes out of scope.
template<typename T, void(*Drop)(T)>
class Owned
{
public:
  explicit Owned(T value)
  : value_(value) {
  }
  ~Owned() {
    Drop(value_);
  }
  Owned(const Owned&) = delete;
  Owned& operator=(const Owned&) = delete;

  const T& operator*() const {
    return value_;
  }
  const T* operator->() const {
    return &value_;
  }

private:
  T value_;
};

using OwnedUpdate = Owned<rv::VoxelsUpdate, rv::drop_update>;
using OwnedVoxelGroups = Owned<rv::VoxelGroups, rv::drop_voxel_groups>;

}  // namespace r3v_ros
