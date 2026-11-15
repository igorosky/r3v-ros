#include "r3v_ros/grid_scene.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <utility>

#include "Eigen/Geometry"
#include "rclcpp/logging.hpp"

#include "r3v_ros/frames.hpp"
#include "r3v_ros/node_common.hpp"

namespace r3v_ros {

namespace {

// A line closer to the boundary than this many cells is the boundary
// (r3v-renderer's BOUNDARY_TOLERANCE).
constexpr double kBoundaryTolerance = 1e-4;

// Lines across an axis: both boundaries and the multiples of the resolution
// between them, or the origin alone on an axis of size 0 (r3v-renderer's
// position_count, in the same float precision).
double positionCount(float size, float resolution) {
  if (size == 0.0f) {
    return 1.0;
  }
  const double half = static_cast<double>(size) / 2.0 / static_cast<double>(resolution);
  const double first = std::floor(-half + kBoundaryTolerance) + 1.0;
  const double last = std::ceil(half - kBoundaryTolerance) - 1.0;
  return 2.0 + std::max(last - first + 1.0, 0.0);
}

std::string describe(size_t index, const r3v_interfaces::msg::Grid& grid) {
  return "grid " + std::to_string(index) + " (frame '" + grid.frame + "')";
}

}  // namespace

uint64_t gridLineCount(const rr::R3vVec3& size, const rr::R3vVec3& resolution) {
  const std::array<float, 3> sizes = { size.x, size.y, size.z };
  const std::array<float, 3> resolutions = { resolution.x, resolution.y, resolution.z };
  std::array<double, 3> counts{ };
  for (size_t axis = 0; axis < 3; ++axis) {
    counts[axis] = positionCount(sizes[axis], resolutions[axis]);
  }
  double lines = 0.0;
  for (size_t axis = 0; axis < 3; ++axis) {
    if (sizes[axis] > 0.0f) {
      lines += counts[(axis + 1) % 3] * counts[(axis + 2) % 3];
    }
  }
  // Rust's `as u64` saturates; a double this large does not fit.
  if (!(lines < 18446744073709551616.0)) {
    return std::numeric_limits<uint64_t>::max();
  }
  return static_cast<uint64_t>(lines);
}

std::optional<std::string> gridsProblem(const r3v_interfaces::msg::GridArray& grids) {
  if (grids.grids.size() > kMaxGrids) {
    return std::to_string(grids.grids.size()) + " grids, at most " + std::to_string(kMaxGrids) +
           " fit";
  }
  for (size_t index = 0; index < grids.grids.size(); ++index) {
    const auto& grid = grids.grids[index];
    const auto& p = grid.pose.position;
    const auto& q = grid.pose.orientation;
    if (!validFloat(p.x) || !validFloat(p.y) || !validFloat(p.z)) {
      return describe(index, grid) + ": position must be finite";
    }
    if (!std::isfinite(q.x) || !std::isfinite(q.y) || !std::isfinite(q.z) ||
      !std::isfinite(q.w) || std::sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w) < 1e-6) {
      return describe(index, grid) + ": orientation must be a finite, non-zero quaternion";
    }
    const auto& s = grid.size;
    if (!validFloat(s.x) || !validFloat(s.y) || !validFloat(s.z) || s.x < 0.0 || s.y < 0.0 ||
      s.z < 0.0) {
      return describe(index, grid) + ": size must be finite and at least 0";
    }
    const auto& r = grid.resolution;
    if (!validPositiveFloat(r.x) || !validPositiveFloat(r.y) || !validPositiveFloat(r.z)) {
      return describe(index, grid) + ": resolution must be positive";
    }
    if (!validColor(grid.color.r, grid.color.g, grid.color.b)) {
      return describe(index, grid) + ": color components must be between 0 and 1";
    }
    const GridSpec spec = gridSpec(grid, "unused");
    const uint64_t lines = gridLineCount(spec.description.size, spec.description.resolution);
    if (lines > kMaxGridLines) {
      return describe(index, grid) + ": " + std::to_string(lines) + " lines, at most " +
             std::to_string(kMaxGridLines) + " fit (make it smaller or coarser)";
    }
  }
  return std::nullopt;
}

GridSpec gridSpec(const r3v_interfaces::msg::Grid& grid, const std::string& fixed_frame) {
  const auto& p = grid.pose.position;
  const auto& q = grid.pose.orientation;
  Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
  pose.translation() = Eigen::Vector3d(p.x, p.y, p.z);
  pose.linear() = Eigen::Quaterniond(q.w, q.x, q.y, q.z).normalized().toRotationMatrix();

  GridSpec spec;
  spec.frame = grid.frame.empty() ? fixed_frame : grid.frame;
  spec.description = rr::r3v_grid_description_default();
  spec.description.size = {
    static_cast<float>(grid.size.x), static_cast<float>(grid.size.y),
    static_cast<float>(grid.size.z) };
  spec.description.resolution = {
    static_cast<float>(grid.resolution.x), static_cast<float>(grid.resolution.y),
    static_cast<float>(grid.resolution.z) };
  spec.description.color = { grid.color.r, grid.color.g, grid.color.b };
  spec.description.frame = nullptr;
  spec.description.pose = toTwist(pose);
  return spec;
}

GridScene::GridScene(rclcpp::Logger logger, rclcpp::Clock::SharedPtr clock)
: logger_(std::move(logger)), clock_(std::move(clock)) {
}

void GridScene::addRenderer(rr::R3vWgpuRenderer* renderer) {
  targets_.push_back({ renderer, { } });
  create(targets_.back());
}

void GridScene::set(std::vector<GridSpec> specs) {
  std::vector<std::string> frames;
  for (const GridSpec& spec : specs) {
    if (std::find(frames.begin(), frames.end(), spec.frame) == frames.end()) {
      frames.push_back(spec.frame);
    }
  }
  for (Target& target : targets_) {
    for (const uint64_t id : target.ids) {
      checkStatus(rr::r3v_wgpu_renderer_remove_grid(target.renderer, id), "removing a grid");
    }
    target.ids.clear();
    // A frame used again later starts unplaced, so its grids wait for TF.
    for (const std::string& frame : frames_) {
      if (std::find(frames.begin(), frames.end(), frame) == frames.end()) {
        checkStatus(
          rr::r3v_wgpu_renderer_remove_frame(target.renderer, frame.c_str()),
          "removing a grid frame");
      }
    }
  }
  specs_ = std::move(specs);
  frames_ = std::move(frames);
  for (Target& target : targets_) {
    create(target);
  }
}

void GridScene::create(Target& target) const {
  for (const GridSpec& spec : specs_) {
    rr::R3vGridDescription description = spec.description;
    description.frame = spec.frame.c_str();  // copied by the renderer
    uint64_t id = 0;
    checkStatus(
      rr::r3v_wgpu_renderer_create_grid(target.renderer, description, &id), "creating a grid");
    target.ids.push_back(id);
  }
}

void GridScene::update(
  const RobotScene& robot, const tf2_ros::Buffer& tf, const std::string& fixed_frame) {
  if (targets_.empty()) {
    return;
  }
  std::string missing;
  for (const std::string& frame : frames_) {
    const auto frame_to_fixed = robot.lookupLatest(tf, fixed_frame, frame);
    if (!frame_to_fixed) {
      missing = frame;
      continue;
    }
    const rr::R3vTwist transform = toTwist(toRender(*frame_to_fixed));
    for (const Target& target : targets_) {
      checkStatus(
        rr::r3v_wgpu_renderer_set_frame_transform(target.renderer, frame.c_str(), transform),
        "placing a grid frame");
    }
  }
  if (!missing.empty()) {
    RCLCPP_WARN_THROTTLE(
      logger_, *clock_, 5000, "No transform from '%s' to '%s' yet; its grids keep their last "
      "pose, or wait for one", missing.c_str(), fixed_frame.c_str());
  }
}

}  // namespace r3v_ros
