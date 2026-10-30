#pragma once

#include <cstdint>
#include <optional>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "Eigen/Geometry"
#include "rclcpp/clock.hpp"
#include "rclcpp/logger.hpp"
#include "urdf_model/link.h"
#if __has_include("tf2_ros/buffer.hpp")
#include "tf2_ros/buffer.hpp"
#else
#include "tf2_ros/buffer.h"
#endif

#include "r3v_ros/mesh_loader.hpp"
#include "r3v_ros/renderer_handles.hpp"

namespace r3v_ros {

// Visuals of a URDF robot as renderer mesh instances, posed from TF.
class RobotScene
{
public:
  RobotScene(
    rclcpp::Logger logger, rclcpp::Clock::SharedPtr clock, MeshLoader loader,
    bool double_sided);

  // Replaces the robot with the one described by `urdf_xml`. The renderer
  // cannot delete assets, so instances of a previous robot are hidden.
  // Throws std::runtime_error when the URDF cannot be parsed.
  void load(const std::string& urdf_xml, rr::R3vWgpuRenderer* renderer);

  // Poses every instance from the latest TF of its link in `fixed_frame`
  // (see lookupLatest()). Instances stay hidden until their link has a
  // transform, and while their link is hidden by setHiddenLinks().
  void update(
    const tf2_ros::Buffer& tf, const std::string& fixed_frame,
    rr::R3vWgpuRenderer* renderer);

  // Transform of `frame` into `fixed_frame` (identity when either is empty or
  // they are equal) from the latest TF. A link of the robot is composed of the
  // latest pose of the URDF root and the latest transform of the link below
  // it: one lookup through both would use their latest common time, which
  // falls out of the TF cache for every moving link when the robot pose is
  // stamped by a lagging clock (a simulator's) and the joints by wall time.
  std::optional<Eigen::Isometry3d> lookupLatest(
    const tf2_ros::Buffer& tf, const std::string& fixed_frame, const std::string& frame,
    std::string* error = nullptr) const;

  // Hides every link in `links` together with all links below it in the URDF
  // tree; the others are shown again. Kept across load(). Takes effect at the
  // next update().
  void setHiddenLinks(const std::vector<std::string>& links);

private:
  struct Instance {
    std::string link;
    Eigen::Isometry3d origin;  // visual in link frame
    uint64_t id;
    bool visible;
  };

  // Names a visual geometry (shape, size, mesh file, scale) in the cache.
  std::string geometryKey(const urdf::Geometry& geometry) const;

  // Triangles of a geometry, built on first use and cached under `key`.
  // NULL (warned about once) when it cannot be built.
  const Triangles * geometryTriangles(const urdf::Geometry& geometry, const std::string& key);

  // Expands hidden_roots_ into hidden_ over children_.
  void updateHidden();

  rclcpp::Logger logger_;
  rclcpp::Clock::SharedPtr clock_;
  MeshLoader loader_;
  bool double_sided_;
  std::vector<Instance> instances_;
  std::string root_;  // root link of the URDF, empty before load()
  std::unordered_map<std::string, std::vector<std::string>> children_;  // link -> child links
  std::vector<std::string> hidden_roots_;
  std::unordered_set<std::string> hidden_;  // hidden_roots_ and every link below them
  std::unordered_set<std::string> unknown_hidden_;  // hidden links already warned about
  std::unordered_map<std::string, Triangles> cache_;
  std::unordered_set<std::string> failed_;  // geometry keys already warned about
};

}  // namespace r3v_ros
