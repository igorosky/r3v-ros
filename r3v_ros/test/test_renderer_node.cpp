#include <gtest/gtest.h>

#include <algorithm>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <thread>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/bool.hpp"

#include "r3v_ros/renderer_node.hpp"
#include "r3v_ros/voxel_stream.hpp"

// The renderer node's lazy view: it keeps the voxel map while nobody follows
// its image, without tesselating it, and draws all of it once someone does.
// Renders on a GPU (a software Vulkan one such as lavapipe works); skipped
// without one.

namespace r3v_ros {
namespace {

using msg::VoxelMapUpdate;
using namespace std::chrono_literals;

constexpr char kVoxelsTopic[] = "/test_renderer_node/voxels";
constexpr char kImageTopic[] = "/test_renderer_node/image";
constexpr char kVisibleTopic[] = "/r3v_renderer/voxels_visible";
constexpr uint64_t kSession = 42;
constexpr double kResolution = 0.1;

// A cube of 16 x 16 x 8 voxels from voxel (x, y, 0).
std::vector<WireBlock> cube(int32_t x, int32_t y) {
  std::vector<WireBlock> blocks;
  for (int32_t dx = 0; dx < 16; dx += 4) {
    for (int32_t dy = 0; dy < 16; dy += 4) {
      for (int32_t z = 0; z < 8; z += 2) {
        blocks.push_back({ x + dx, y + dy, z, 0xffffffffu });
      }
    }
  }
  return blocks;
}

// Pixels of a bgra8 `image` brighter than its black background.
size_t litPixels(const sensor_msgs::msg::Image& image) {
  size_t lit = 0;
  for (size_t i = 0; i + 3 < image.data.size(); i += 4) {
    if (std::max({ image.data[i], image.data[i + 1], image.data[i + 2] }) > 16) {
      ++lit;
    }
  }
  return lit;
}

class RendererNodeTest : public ::testing::Test {
protected:
  static void SetUpTestSuite() {
    rclcpp::init(0, nullptr);
  }

  static void TearDownTestSuite() {
    rclcpp::shutdown();
  }

  void SetUp() override {
    std::vector<rclcpp::Parameter> overrides;
    overrides.emplace_back("rate", 30.0);
    overrides.emplace_back("voxel_stream.topic", kVoxelsTopic);
    overrides.emplace_back("voxel_stream.request_topic", "/test_renderer_node/request_full_sync");
    overrides.emplace_back("robot_description.topic", "");
    overrides.emplace_back("output.topic", kImageTopic);
    overrides.emplace_back("output.width", 64);
    overrides.emplace_back("output.height", 48);
    overrides.emplace_back("output.encoding", "bgra8");
    overrides.emplace_back("output.qos", "reliable");
    overrides.emplace_back("output.lazy", true);
    rclcpp::NodeOptions options;
    options.parameter_overrides(overrides);
    try {
      renderer_ = std::make_shared<RendererNode>(options);
    } catch (const std::exception& e) {
      GTEST_SKIP() << "No renderer: " << e.what();
    }
    peer_ = std::make_shared<rclcpp::Node>("test_renderer_node_peer");
    voxels_pub_ = peer_->create_publisher<VoxelMapUpdate>(kVoxelsTopic, rclcpp::QoS(100).reliable());
    visible_pub_ = peer_->create_publisher<std_msgs::msg::Bool>(kVisibleTopic, rclcpp::QoS(10));
    executor_ = std::make_unique<rclcpp::executors::MultiThreadedExecutor>();
    executor_->add_node(renderer_);
    executor_->add_node(peer_);
    spinner_ = std::thread([this]() { executor_->spin(); });
  }

  void TearDown() override {
    if (executor_) {
      executor_->cancel();
    }
    if (spinner_.joinable()) {
      spinner_.join();
    }
  }

  // Waits up to `timeout` for `done`.
  static bool waitFor(const std::function<bool()>& done, std::chrono::milliseconds timeout) {
    const auto end = std::chrono::steady_clock::now() + timeout;
    while (!done()) {
      if (std::chrono::steady_clock::now() > end) {
        return false;
      }
      std::this_thread::sleep_for(20ms);
    }
    return true;
  }

  void publish(uint8_t kind, const std::vector<WireBlock>& blocks) {
    VoxelMapUpdate msg;
    msg.header.frame_id = "map";
    msg.session = kSession;
    msg.sequence = sequence_++;
    msg.kind = kind;
    msg.compression = VoxelMapUpdate::COMPRESSION_NONE;
    msg.resolution = kResolution;
    if (!blocks.empty()) {
      msg.data = encodePayload(
        blocks.data(), blocks.size() * sizeof(WireBlock), VoxelMapUpdate::COMPRESSION_NONE, 0);
    }
    voxels_pub_->publish(msg);
  }

  // An empty FULL message, then `blocks`, once the renderer follows the
  // stream.
  void sendMap(const std::vector<WireBlock>& blocks) {
    ASSERT_TRUE(waitFor([this]() { return voxels_pub_->get_subscription_count() > 0; }, 10s));
    publish(VoxelMapUpdate::FULL, { });
    publish(VoxelMapUpdate::DELTA, blocks);
  }

  void sendBlocks(const std::vector<WireBlock>& blocks) {
    publish(VoxelMapUpdate::DELTA, blocks);
  }

  void setVisible(bool visible) {
    std_msgs::msg::Bool msg;
    msg.data = visible;
    visible_pub_->publish(msg);
  }

  // Subscribes to the image.
  void follow() {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      frames_.clear();
    }
    image_sub_ = peer_->create_subscription<sensor_msgs::msg::Image>(
      kImageTopic, rclcpp::QoS(10).reliable(),
      [this](sensor_msgs::msg::Image::ConstSharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        frames_.push_back(litPixels(*msg));
      });
  }

  // Unsubscribes from the image, and waits until the renderer is left alone.
  void unfollow() {
    image_sub_.reset();
    ASSERT_TRUE(waitFor([this]() { return peer_->count_subscribers(kImageTopic) == 0; }, 10s));
    // A few frames, so that the renderer notices.
    std::this_thread::sleep_for(300ms);
  }

  // Waits for a frame whose lit pixel count passes `wanted`, and returns that
  // count (0 when none came).
  size_t frameWith(const std::function<bool(size_t)>& wanted) {
    size_t found = 0;
    waitFor(
      [&]() {
        std::lock_guard<std::mutex> lock(mutex_);
        for (const size_t lit : frames_) {
          if (wanted(lit)) {
            found = lit;
            return true;
          }
        }
        frames_.clear();
        return false;
      }, 10s);
    return found;
  }

  std::shared_ptr<RendererNode> renderer_;
  rclcpp::Node::SharedPtr peer_;
  rclcpp::Publisher<VoxelMapUpdate>::SharedPtr voxels_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr visible_pub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  std::unique_ptr<rclcpp::executors::MultiThreadedExecutor> executor_;
  std::thread spinner_;
  uint64_t sequence_ = 0;
  std::mutex mutex_;
  std::vector<size_t> frames_;  // lit pixels of every frame since the last look
};

const auto kSomeVoxels = [](size_t lit) { return lit > 50; };

TEST_F(RendererNodeTest, VoxelsArrivingUnfollowedShowOnceFollowed) {
  sendMap(cube(-8, -8));
  // Rendered (and tesselated) by nobody meanwhile.
  std::this_thread::sleep_for(500ms);
  follow();
  EXPECT_GT(frameWith(kSomeVoxels), 0u);
}

TEST_F(RendererNodeTest, ChangesWhileUnfollowedShowOnceFollowedAgain) {
  sendMap(cube(-8, -8));
  follow();
  const size_t one_cube = frameWith(kSomeVoxels);
  ASSERT_GT(one_cube, 0u);
  unfollow();
  // Beside the first one, in view of the default camera.
  sendBlocks(cube(12, -28));
  std::this_thread::sleep_for(300ms);
  follow();
  EXPECT_GT(frameWith([one_cube](size_t lit) { return lit > one_cube * 5 / 4; }), 0u)
    << "the cube sent while nobody followed the image is missing";
}

TEST_F(RendererNodeTest, HiddenVoxelsComeBack) {
  sendMap(cube(-8, -8));
  follow();
  ASSERT_GT(frameWith(kSomeVoxels), 0u);
  setVisible(false);
  EXPECT_TRUE(
    waitFor(
      [this]() {
        std::lock_guard<std::mutex> lock(mutex_);
        const bool empty = std::find(frames_.begin(), frames_.end(), 0u) != frames_.end();
        frames_.clear();
        return empty;
      }, 10s)) << "hidden voxels still drawn";
  setVisible(true);
  EXPECT_GT(frameWith(kSomeVoxels), 0u);
}

}  // namespace
}  // namespace r3v_ros
