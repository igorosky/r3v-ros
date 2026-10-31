#pragma once

#include "Eigen/Geometry"

#include "r3v_ros/renderer_handles.hpp"

namespace r3v_ros {

// Where the virtual camera is, in the fixed frame (ROS, Z-up).
struct CameraView {
  Eigen::Vector3d eye;
  Eigen::Vector3d target;
  Eigen::Vector3d up;
};

struct CameraLens {
  double fov_deg;  // vertical field of view
  double near;
  double far;
};

// Right-handed look-at view and perspective projection with wgpu's 0..1
// depth range, built in the renderer world. Returns false for a degenerate
// view (eye on target).
bool makeCameraState(
  const CameraView& view, const CameraLens& lens, double aspect_ratio,
  rr::R3vCameraState& out);

}  // namespace r3v_ros
