#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include "r3v_interfaces/msg/grid.hpp"
#include "r3v_interfaces/msg/grid_array.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/logger.hpp"
#if __has_include("tf2_ros/buffer.hpp")
#include "tf2_ros/buffer.hpp"
#else
#include "tf2_ros/buffer.h"
#endif

#include "r3v_ros/renderer_handles.hpp"
#include "r3v_ros/robot_scene.hpp"

namespace r3v_ros {

// Most grids one ~/grids/set may hold.
constexpr size_t kMaxGrids = 64;
// Most lines one grid may have (r3v-renderer's MAX_GRID_LINES).
constexpr uint64_t kMaxGridLines = uint64_t{ 1 } << 20;

// Lines of a grid, exactly as r3v-renderer counts them (grid::line_count),
// without building it. Saturates at UINT64_MAX.
uint64_t gridLineCount(const rr::R3vVec3& size, const rr::R3vVec3& resolution);

// Why the renderer cannot draw `grids`, naming the first bad grid, or nothing
// when it can.
std::optional<std::string> gridsProblem(const r3v_interfaces::msg::GridArray& grids);

// A grid as the renderer takes it. description.frame is left NULL: it points
// into `frame` when the grid is created.
struct GridSpec {
  std::string frame;
  rr::R3vGridDescription description;
};

// `grid` (one gridsProblem() accepts) for the renderer: an empty frame is
// `fixed_frame`, the orientation is normalized and the color's alpha dropped.
// The pose stays in the frame's own (ROS) axes: the frame's transform brings
// it into the renderer's Y-up world.
GridSpec gridSpec(const r3v_interfaces::msg::Grid& grid, const std::string& fixed_frame);

// Grids of any number of renderers, each placed in a TF frame that update()
// poses.
class GridScene
{
public:
  GridScene(rclcpp::Logger logger, rclcpp::Clock::SharedPtr clock);

  // Draws the grids with `renderer` too. `renderer` has to outlive the scene.
  void addRenderer(rr::R3vWgpuRenderer* renderer);

  // Replaces every grid in every renderer.
  void set(std::vector<GridSpec> specs);

  // Places the frames of the grids from the latest TF (RobotScene::lookupLatest,
  // so robot links are posed exactly as the robot's visuals). A frame without
  // a transform keeps its last pose; its grids are not drawn until it first
  // has one.
  void update(const RobotScene& robot, const tf2_ros::Buffer& tf, const std::string& fixed_frame);

private:
  struct Target {
    rr::R3vWgpuRenderer* renderer;
    std::vector<uint64_t> ids;  // as specs_
  };

  // Creates specs_ in `target`.
  void create(Target& target) const;

  rclcpp::Logger logger_;
  rclcpp::Clock::SharedPtr clock_;
  std::vector<GridSpec> specs_;
  std::vector<std::string> frames_;  // distinct frames of specs_
  std::vector<Target> targets_;
};

}  // namespace r3v_ros
