#include "r3v_ros/camera.hpp"

#include <cmath>
#include <cstring>

#include "r3v_ros/frames.hpp"

namespace r3v_ros {

bool makeCameraState(
  const CameraView& view, const CameraLens& lens, double aspect_ratio,
  rr::R3vCameraState& out) {
  const Eigen::Vector3d eye = toRender(view.eye);
  const Eigen::Vector3d target = toRender(view.target);
  Eigen::Vector3d up = toRender(view.up);

  Eigen::Vector3d forward = target - eye;
  if (forward.norm() < 1e-9) {
    return false;
  }
  forward.normalize();
  if (up.norm() < 1e-9 || std::abs(forward.dot(up.normalized())) > 0.9999) {
    // Looking straight along up (e.g. top down): fixed frame +X becomes up.
    up = std::abs(forward.x()) < 0.9 ? Eigen::Vector3d::UnitX() : Eigen::Vector3d::UnitZ();
  }
  const Eigen::Vector3d right = forward.cross(up).normalized();
  const Eigen::Vector3d true_up = right.cross(forward);

  Eigen::Matrix4d view_matrix = Eigen::Matrix4d::Identity();
  view_matrix.block<1, 3>(0, 0) = right.transpose();
  view_matrix.block<1, 3>(1, 0) = true_up.transpose();
  view_matrix.block<1, 3>(2, 0) = -forward.transpose();
  view_matrix(0, 3) = -right.dot(eye);
  view_matrix(1, 3) = -true_up.dot(eye);
  view_matrix(2, 3) = forward.dot(eye);

  // OpenGL style perspective, then remapped from -1..1 to 0..1 depth.
  const double f = 1.0 / std::tan(lens.fov_deg * M_PI / 360.0);
  Eigen::Matrix4d projection = Eigen::Matrix4d::Zero();
  projection(0, 0) = f / aspect_ratio;
  projection(1, 1) = f;
  projection(2, 2) = (lens.far + lens.near) / (lens.near - lens.far);
  projection(2, 3) = 2.0 * lens.far * lens.near / (lens.near - lens.far);
  projection(3, 2) = -1.0;
  Eigen::Matrix4d opengl_to_wgpu = Eigen::Matrix4d::Identity();
  opengl_to_wgpu(2, 2) = 0.5;
  opengl_to_wgpu(2, 3) = 0.5;

  // Eigen is column-major like view_proj[column][row].
  const Eigen::Matrix4f view_proj = (opengl_to_wgpu * projection * view_matrix).cast<float>();
  std::memcpy(out.view_proj, view_proj.data(), sizeof(out.view_proj));
  out.position = { static_cast<float>(eye.x()), static_cast<float>(eye.y()),
    static_cast<float>(eye.z()) };
  out.forward = { static_cast<float>(forward.x()), static_cast<float>(forward.y()),
    static_cast<float>(forward.z()) };
  return true;
}

}  // namespace r3v_ros
