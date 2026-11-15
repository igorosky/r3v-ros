#pragma once

#include <cmath>

#include "Eigen/Geometry"

namespace r3v_ros {

// The renderer world is Y-up (its camera and lights assume it) while ROS is
// Z-up. A rotation of -90 degrees about X maps ROS coordinates to renderer
// coordinates: (x, y, z) -> (x, z, -y). It only permutes axes, so voxel grids
// stay aligned.

inline Eigen::Quaterniond renderRotation() {
  return Eigen::Quaterniond(Eigen::AngleAxisd(-M_PI / 2.0, Eigen::Vector3d::UnitX()));
}

// Point or direction from the fixed frame to the renderer world.
inline Eigen::Vector3d toRender(const Eigen::Vector3d& v) {
  return { v.x(), v.z(), -v.y() };
}

// Pose (local -> fixed frame) to a pose local -> renderer world.
inline Eigen::Isometry3d toRender(const Eigen::Isometry3d& pose) {
  return Eigen::Isometry3d(renderRotation()) * pose;
}

}  // namespace r3v_ros
