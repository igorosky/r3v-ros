#include "r3v_ros/renderer_handles.hpp"

#include <stdexcept>

namespace r3v_ros {

std::string lastRendererError() {
  const char* message = rr::r3v_renderer_last_error();
  return message != nullptr ? message : "";
}

void checkStatus(rr::R3vStatus status, const std::string& what) {
  if (status != rr::R3vStatus::R3V_OK) {
    throw std::runtime_error(
            what + " failed (status " + std::to_string(static_cast<int>(status)) + "): " +
            lastRendererError());
  }
}

rr::R3vTwist toTwist(const Eigen::Isometry3d& pose) {
  const Eigen::Quaterniond q(pose.linear());
  const Eigen::Vector3d t = pose.translation();
  return {
    { static_cast<float>(t.x()), static_cast<float>(t.y()), static_cast<float>(t.z()) },
    { static_cast<float>(q.x()), static_cast<float>(q.y()), static_cast<float>(q.z()),
      static_cast<float>(q.w()) } };
}

}  // namespace r3v_ros
