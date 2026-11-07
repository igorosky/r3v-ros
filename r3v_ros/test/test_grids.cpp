#include <gtest/gtest.h>

#include <chrono>
#include <cstdint>
#include <limits>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "rclcpp/rclcpp.hpp"

#include "r3v_ros/grid_scene.hpp"
#include "r3v_ros/renderer_node.hpp"

// The renderer node's grids: line counts as r3v-renderer has them, which
// arrays it takes, and ~/grids/set and ~/grids/get of a node rendering
// nothing (no GPU needed).

namespace r3v_ros {
namespace {

using r3v_interfaces::msg::Grid;
using r3v_interfaces::msg::GridArray;

rr::R3vVec3 vec(float x, float y, float z) {
  return { x, y, z };
}

// A 10 x 10 m plane of 1 m cells in `frame`.
Grid plane(const std::string& frame) {
  Grid grid;
  grid.frame = frame;
  grid.pose.orientation.w = 1.0;
  grid.size.x = 10.0;
  grid.size.y = 10.0;
  grid.resolution.x = 1.0;
  grid.resolution.y = 1.0;
  grid.resolution.z = 1.0;
  grid.color.r = 1.0f;
  grid.color.g = 0.5f;
  grid.color.a = 1.0f;
  return grid;
}

GridArray array(std::vector<Grid> grids) {
  GridArray out;
  out.grids = std::move(grids);
  return out;
}

// The problem of an array of `grid` alone.
std::string problem(const Grid& grid) {
  return gridsProblem(array({ grid })).value_or("");
}

TEST(GridLineCount, CountsAsTheRenderer) {
  const auto ones = vec(1.0f, 1.0f, 1.0f);
  EXPECT_EQ(gridLineCount(vec(2.0f, 0.0f, 2.0f), ones), 6u);
  EXPECT_EQ(gridLineCount(vec(2.0f, 2.0f, 2.0f), ones), 27u);
  EXPECT_EQ(gridLineCount(vec(4.0f, 2.0f, 0.0f), vec(2.0f, 0.5f, 1.0f)), 8u);
  EXPECT_EQ(gridLineCount(vec(0.0f, 0.0f, 0.0f), ones), 0u);
  // 3 m at 0.1: 31 positions, the ones next to the boundary are it
  EXPECT_EQ(gridLineCount(vec(3.0f, 3.0f, 0.0f), vec(0.1f, 0.1f, 0.1f)), 2u * 31u);
  EXPECT_EQ(gridLineCount(vec(1e6f, 0.0f, 1e6f), ones), 2u * 1000001u);
  const float max = std::numeric_limits<float>::max();
  const float min = std::numeric_limits<float>::min();
  EXPECT_EQ(
    gridLineCount(vec(max, max, max), vec(min, min, min)), std::numeric_limits<uint64_t>::max());
}

TEST(GridsProblem, TakesValidGrids) {
  EXPECT_EQ(gridsProblem(array({ })), std::nullopt);
  Grid lattice = plane("");
  lattice.size.z = 4.0;
  lattice.pose.position.x = -3.0;
  lattice.pose.orientation.z = 1.0;  // normalized by gridSpec
  EXPECT_EQ(gridsProblem(array({ plane("base_footprint"), lattice })), std::nullopt);
}

TEST(GridsProblem, NamesTheBadGrid) {
  Grid bad = plane("gripper");
  bad.size.x = -1.0;
  const auto message = gridsProblem(array({ plane("map"), bad }));
  ASSERT_TRUE(message.has_value());
  EXPECT_NE(message->find("grid 1 (frame 'gripper')"), std::string::npos) << *message;
  EXPECT_NE(message->find("size"), std::string::npos) << *message;
}

TEST(GridsProblem, RejectsWhatTheRendererCannotDraw) {
  const double nan = std::numeric_limits<double>::quiet_NaN();
  Grid grid = plane("map");
  grid.size.y = nan;
  EXPECT_NE(problem(grid).find("size"), std::string::npos);
  grid = plane("map");
  grid.pose.position.z = std::numeric_limits<double>::infinity();
  EXPECT_NE(problem(grid).find("position"), std::string::npos);
  grid = plane("map");
  grid.pose.orientation.w = 0.0;
  EXPECT_NE(problem(grid).find("orientation"), std::string::npos);
  grid = plane("map");
  grid.resolution.y = 0.0;
  EXPECT_NE(problem(grid).find("resolution"), std::string::npos);
  grid = plane("map");
  grid.color.g = 1.5f;
  EXPECT_NE(problem(grid).find("color"), std::string::npos);
  grid = plane("map");
  grid.size.x = 2000.0;
  grid.size.y = 2000.0;
  grid.size.z = 2000.0;
  EXPECT_NE(problem(grid).find("lines"), std::string::npos);
  std::vector<Grid> many(kMaxGrids + 1, plane("map"));
  EXPECT_NE(gridsProblem(array(many)).value_or("").find("at most 64"), std::string::npos);
}

TEST(GridSpec, ConvertsForTheRenderer) {
  Grid grid = plane("");
  grid.pose.position.x = 1.0;
  grid.pose.orientation.w = 2.0;  // not normalized
  grid.color.a = 0.0f;
  const GridSpec spec = gridSpec(grid, "map");
  EXPECT_EQ(spec.frame, "map");
  EXPECT_EQ(spec.description.frame, nullptr);
  EXPECT_FLOAT_EQ(spec.description.size.x, 10.0f);
  EXPECT_FLOAT_EQ(spec.description.size.z, 0.0f);
  EXPECT_FLOAT_EQ(spec.description.color.x, 1.0f);
  EXPECT_FLOAT_EQ(spec.description.color.y, 0.5f);
  EXPECT_FLOAT_EQ(spec.description.pose.linear.x, 1.0f);
  EXPECT_FLOAT_EQ(spec.description.pose.rotational.w, 1.0f);
  EXPECT_EQ(gridSpec(plane("base_footprint"), "map").frame, "base_footprint");
}

class RendererNodeGridsTest : public ::testing::Test {
protected:
  static void SetUpTestSuite() {
    rclcpp::init(0, nullptr);
  }

  static void TearDownTestSuite() {
    rclcpp::shutdown();
  }

  void SetUp() override {
    rclcpp::NodeOptions options;
    // Renders nothing, so no GPU, while its controls still apply.
    options.parameter_overrides({ rclcpp::Parameter("enabled", false) });
    options.arguments({ "--ros-args", "-r", "__node:=test_r3v_grids" });
    renderer_ = std::make_shared<RendererNode>(options);
    client_ = std::make_shared<rclcpp::Node>("test_r3v_grids_client");
    set_ = client_->create_publisher<GridArray>("/test_r3v_grids/grids/set", rclcpp::QoS(10));
    get_ = client_->create_subscription<GridArray>(
      "/test_r3v_grids/grids/get", rclcpp::QoS(1).reliable().transient_local(),
      [this](GridArray::ConstSharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        received_.push_back(*msg);
      });
    executor_.add_node(renderer_);
    executor_.add_node(client_);
  }

  // Spins until ~/grids/get has said `count` arrays, or `timeout`. Every
  // array it said by then.
  std::vector<GridArray> received(size_t count, std::chrono::milliseconds timeout) {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < deadline) {
      executor_.spin_some(std::chrono::milliseconds(20));
      std::lock_guard<std::mutex> lock(mutex_);
      if (received_.size() >= count) {
        break;
      }
    }
    std::lock_guard<std::mutex> lock(mutex_);
    return received_;
  }

  void set(const GridArray& grids) {
    // Until the renderer's subscription matched, a set would be lost.
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(10);
    while (set_->get_subscription_count() == 0 && std::chrono::steady_clock::now() < deadline) {
      executor_.spin_some(std::chrono::milliseconds(20));
    }
    set_->publish(grids);
  }

  std::shared_ptr<RendererNode> renderer_;
  std::shared_ptr<rclcpp::Node> client_;
  rclcpp::Publisher<GridArray>::SharedPtr set_;
  rclcpp::Subscription<GridArray>::SharedPtr get_;
  rclcpp::executors::SingleThreadedExecutor executor_;
  std::mutex mutex_;
  std::vector<GridArray> received_;
};

TEST_F(RendererNodeGridsTest, StartsEmptyAndEchoesWhatItDraws) {
  auto said = received(1, std::chrono::seconds(10));
  ASSERT_EQ(said.size(), 1u);
  EXPECT_TRUE(said[0].grids.empty());

  const GridArray grids = array({ plane("base_footprint"), plane("") });
  set(grids);
  said = received(2, std::chrono::seconds(10));
  ASSERT_EQ(said.size(), 2u);
  // As sent: the empty frame stays empty
  EXPECT_EQ(said[1], grids);

  // The same grids again change nothing, invalid ones are ignored whole
  set(grids);
  GridArray invalid = array({ plane("map"), plane("map") });
  invalid.grids[1].resolution.x = -1.0;
  set(invalid);
  said = received(3, std::chrono::seconds(1));
  EXPECT_EQ(said.size(), 2u);

  set(array({ }));
  said = received(3, std::chrono::seconds(10));
  ASSERT_EQ(said.size(), 3u);
  EXPECT_TRUE(said[2].grids.empty());
}

}  // namespace
}  // namespace r3v_ros
