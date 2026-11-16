#include <gtest/gtest.h>

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <tuple>
#include <utility>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/point_field.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float64.hpp"

#include "r3v_ros/voxel_map.hpp"
#include "r3v_ros/voxel_stream.hpp"
#include "r3v_ros/voxelizer_node.hpp"

// The voxelizer node's throttled stream: the downsampled map, whose blocks go
// out together at most `throttle.rate` times a second while the throttle is
// on, and right away while it is off. FULL messages are never held back,
// heartbeats go out while blocks wait only when the throttle period is longer
// than theirs. Runs the single_threaded backend on point clouds published
// here, no GPU needed.

namespace r3v_ros {
namespace {

using msg::FullSyncRequest;
using msg::VoxelMapUpdate;
using Clock = std::chrono::steady_clock;

constexpr char kCloudTopic[] = "/test_throttled_stream/cloud";
constexpr char kDownsampledTopic[] = "/test_throttled_stream/voxels_downsampled";
constexpr char kThrottledTopic[] = "/test_throttled_stream/voxels_throttled";
constexpr double kStreamRate = 20.0;

// Whether `a` and `b` hold the same voxels (VoxelMap::groups() has no order).
bool sameMap(std::vector<WireGroup> a, std::vector<WireGroup> b) {
  const auto by_key = [](const WireGroup& l, const WireGroup& r) {
      return std::tie(l.key.x, l.key.y, l.key.z) < std::tie(r.key.x, r.key.y, r.key.z);
    };
  std::sort(a.begin(), a.end(), by_key);
  std::sort(b.begin(), b.end(), by_key);
  return a.size() == b.size() &&
         std::equal(
    a.begin(), a.end(), b.begin(), [](const WireGroup& l, const WireGroup& r) {
      return l.key.x == r.key.x && l.key.y == r.key.y && l.key.z == r.key.z &&
      std::memcmp(l.pages, r.pages, sizeof(l.pages)) == 0;
    });
}

// A wall of points at x = `x` m, y in [-10, 10] m and z in [-5, 5] m, in the
// fixed frame, seen from its origin.
sensor_msgs::msg::PointCloud2 wall(float x) {
  sensor_msgs::msg::PointCloud2 cloud;
  cloud.header.frame_id = "map";
  for (const char* name : { "x", "y", "z" }) {
    sensor_msgs::msg::PointField field;
    field.name = name;
    field.offset = static_cast<uint32_t>(cloud.fields.size() * sizeof(float));
    field.datatype = sensor_msgs::msg::PointField::FLOAT32;
    field.count = 1;
    cloud.fields.push_back(field);
  }
  cloud.point_step = 3 * sizeof(float);
  cloud.height = 1;
  std::vector<float> points;
  for (int y = -10; y <= 10; ++y) {
    for (int z = -5; z <= 5; ++z) {
      points.insert(points.end(), { x + 0.5f, static_cast<float>(y) + 0.5f,
          static_cast<float>(z) + 0.5f });
    }
  }
  cloud.width = static_cast<uint32_t>(points.size() / 3);
  cloud.row_step = cloud.width * cloud.point_step;
  cloud.data.resize(points.size() * sizeof(float));
  std::memcpy(cloud.data.data(), points.data(), cloud.data.size());
  return cloud;
}

// One stream as a renderer follows it: every message it got, and the map they
// make.
class Follower
{
public:
  struct Message {
    Clock::time_point received;
    rclcpp::Time stamp;
    uint64_t session;
    uint64_t sequence;
    uint8_t kind;
    size_t blocks;  // DELTA only
  };

  Follower(rclcpp::Node& node, const std::string& topic) {
    sub_ = node.create_subscription<VoxelMapUpdate>(
      topic, rclcpp::QoS(100).reliable(),
      [this](VoxelMapUpdate::ConstSharedPtr msg) { onMessage(*msg); });
  }

  std::vector<Message> messages() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return messages_;
  }

  std::vector<WireGroup> groups() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return map_.groups();
  }

  bool synced() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return synced_;
  }

  // DELTA messages with blocks.
  std::vector<Message> blockDeltas() const {
    std::vector<Message> deltas;
    for (const Message& message : messages()) {
      if (message.kind == VoxelMapUpdate::DELTA && message.blocks > 0) {
        deltas.push_back(message);
      }
    }
    return deltas;
  }

private:
  void onMessage(const VoxelMapUpdate& msg) {
    std::lock_guard<std::mutex> lock(mutex_);
    Message message{ Clock::now(), rclcpp::Time(msg.header.stamp), msg.session, msg.sequence,
      msg.kind, 0 };
    if (msg.kind == VoxelMapUpdate::FULL) {
      map_.replace(decodePayload<WireGroup>(msg));
      synced_ = true;
    } else {
      const std::vector<WireBlock> blocks = decodePayload<WireBlock>(msg);
      message.blocks = blocks.size();
      if (synced_) {
        map_.apply(blocks);
      }
    }
    messages_.push_back(message);
  }

  mutable std::mutex mutex_;
  std::vector<Message> messages_;
  VoxelMap map_;
  bool synced_ = false;
  rclcpp::Subscription<VoxelMapUpdate>::SharedPtr sub_;
};

class ThrottledStreamTest : public ::testing::Test {
protected:
  static void SetUpTestSuite() {
    rclcpp::init(0, nullptr);
  }

  static void TearDownTestSuite() {
    rclcpp::shutdown();
  }

  // A voxelizer at 1 m on clouds from kCloudTopic, with this test's stream
  // topics, plus `overrides`. One hit makes a voxel.
  void start(std::vector<rclcpp::Parameter> overrides) {
    overrides.emplace_back("voxelizer.backend", "single_threaded");
    overrides.emplace_back("voxelizer.resolution", 1.0);
    overrides.emplace_back("voxel_stream.topic", "/test_throttled_stream/voxels");
    overrides.emplace_back("voxel_stream.downsampled_topic", kDownsampledTopic);
    overrides.emplace_back("voxel_stream.throttled_topic", kThrottledTopic);
    overrides.emplace_back("voxel_stream.rate", kStreamRate);
    overrides.emplace_back("point_clouds.names", std::vector<std::string>{ "test" });
    overrides.emplace_back("point_clouds.test.topic", kCloudTopic);
    overrides.emplace_back("point_clouds.test.qos", "reliable");
    overrides.emplace_back("point_clouds.test.max_range", 100.0);
    rclcpp::NodeOptions options;
    options.parameter_overrides(overrides);
    voxelizer_ = std::make_shared<VoxelizerNode>(options);
    client_ = std::make_shared<rclcpp::Node>("test_throttled_stream_client");
    cloud_pub_ =
      client_->create_publisher<sensor_msgs::msg::PointCloud2>(kCloudTopic, rclcpp::QoS(10));
    executor_ = std::make_unique<rclcpp::executors::MultiThreadedExecutor>();
    executor_->add_node(voxelizer_);
    executor_->add_node(client_);
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
  static bool waitFor(const std::function<bool()>& done, std::chrono::duration<double> timeout) {
    const auto deadline = Clock::now() + timeout;
    while (!done()) {
      if (Clock::now() > deadline) {
        return false;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    return true;
  }

  // Publishes walls at `count` places, `period` apart.
  void publishWalls(int count, std::chrono::milliseconds period) {
    for (int i = 0; i < count; ++i) {
      cloud_pub_->publish(wall(10.0f + 4.0f * static_cast<float>(i)));
      std::this_thread::sleep_for(period);
    }
  }

  std::shared_ptr<VoxelizerNode> voxelizer_;
  std::shared_ptr<rclcpp::Node> client_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
  std::unique_ptr<rclcpp::executors::MultiThreadedExecutor> executor_;
  std::thread spinner_;
};

TEST_F(ThrottledStreamTest, MirrorsTheDownsampledStreamWhileOff) {
  start({ });
  Follower downsampled(*client_, kDownsampledTopic);
  Follower throttled(*client_, kThrottledTopic);
  // The first subscriber of a stream gets a FULL message unasked.
  ASSERT_TRUE(
    waitFor(
      [&]() { return downsampled.synced() && throttled.synced(); }, std::chrono::seconds(10)));
  ASSERT_TRUE(
    waitFor(
      [&]() { return cloud_pub_->get_subscription_count() > 0; }, std::chrono::seconds(10)));

  publishWalls(5, std::chrono::milliseconds(150));
  ASSERT_TRUE(
    waitFor(
      [&]() {
        return !downsampled.groups().empty() &&
        sameMap(downsampled.groups(), throttled.groups());
      }, std::chrono::seconds(5)));
  std::this_thread::sleep_for(std::chrono::milliseconds(300));

  // Every tick sends both streams the same blocks.
  EXPECT_GE(downsampled.blockDeltas().size(), 2u);
  EXPECT_EQ(throttled.blockDeltas().size(), downsampled.blockDeltas().size());
  EXPECT_TRUE(sameMap(downsampled.groups(), throttled.groups()));
}

TEST_F(ThrottledStreamTest, SendsTheBlocksTogetherWhileOn) {
  constexpr double kRate = 2.0;
  start({ rclcpp::Parameter("throttle.enabled", true), rclcpp::Parameter("throttle.rate", kRate) });
  Follower downsampled(*client_, kDownsampledTopic);
  Follower throttled(*client_, kThrottledTopic);
  ASSERT_TRUE(
    waitFor(
      [&]() { return downsampled.synced() && throttled.synced(); }, std::chrono::seconds(10)));
  ASSERT_TRUE(
    waitFor(
      [&]() { return cloud_pub_->get_subscription_count() > 0; }, std::chrono::seconds(10)));

  // 20 walls in 1.5 s.
  publishWalls(20, std::chrono::milliseconds(75));
  ASSERT_TRUE(
    waitFor(
      [&]() {
        return !downsampled.groups().empty() &&
        sameMap(downsampled.groups(), throttled.groups());
      }, std::chrono::seconds(5)));

  const std::vector<Follower::Message> fast = downsampled.blockDeltas();
  const std::vector<Follower::Message> slow = throttled.blockDeltas();
  ASSERT_FALSE(slow.empty());
  // About 20 against about 4.
  EXPECT_GE(fast.size(), 10u);
  EXPECT_LE(slow.size(), 6u);
  // Ticks come every 1 / kStreamRate; a message with blocks waits for the
  // tick at least 1 / kRate after the previous one.
  const double min_gap = 1.0 / kRate - 0.5 / kStreamRate - 0.005;
  for (size_t i = 1; i < slow.size(); ++i) {
    EXPECT_GE((slow[i].stamp - slow[i - 1].stamp).seconds(), min_gap) << "message " << i;
  }
  // Blocks waited all along, never longer than a heartbeat period: no
  // heartbeat in between.
  for (const Follower::Message& message : throttled.messages()) {
    if (message.stamp > slow.front().stamp && message.stamp < slow.back().stamp) {
      EXPECT_GT(message.blocks, 0u) << "message " << message.sequence;
    }
  }
}

TEST_F(ThrottledStreamTest, FullMessagesAndHeartbeatsOfLongWaitsGoOut) {
  // One message with blocks every 20 s at most.
  start({ rclcpp::Parameter("throttle.enabled", true), rclcpp::Parameter("throttle.rate", 0.05) });
  Follower throttled(*client_, kThrottledTopic);
  ASSERT_TRUE(waitFor([&]() { return throttled.synced(); }, std::chrono::seconds(10)));
  ASSERT_TRUE(
    waitFor(
      [&]() { return cloud_pub_->get_subscription_count() > 0; }, std::chrono::seconds(10)));
  auto request_pub = client_->create_publisher<FullSyncRequest>(
    "/r3v_voxelizer/throttled/request_full_sync", rclcpp::QoS(10));
  ASSERT_TRUE(
    waitFor(
      [&]() { return request_pub->get_subscription_count() > 0; }, std::chrono::seconds(10)));

  publishWalls(1, std::chrono::milliseconds(0));
  // The blocks wait longer than a heartbeat period, so heartbeats (empty
  // DELTAs) keep coming every second.
  const size_t before = throttled.messages().size();
  std::this_thread::sleep_for(std::chrono::milliseconds(2500));
  const std::vector<Follower::Message> messages = throttled.messages();
  EXPECT_GE(messages.size() - before, 2u);
  EXPECT_TRUE(throttled.blockDeltas().empty());
  EXPECT_TRUE(throttled.groups().empty());

  // A FULL message goes out at the next tick and brings the wall.
  FullSyncRequest request;
  request.session = messages.back().session;
  request.since = messages.back().sequence + 1;
  const auto asked = Clock::now();
  request_pub->publish(request);
  ASSERT_TRUE(
    waitFor(
      [&]() {
        const std::vector<Follower::Message> now = throttled.messages();
        return std::any_of(
          now.begin(), now.end(), [&](const Follower::Message& message) {
            return message.kind == VoxelMapUpdate::FULL && message.sequence >= request.since;
          });
      }, std::chrono::seconds(2)));
  EXPECT_LT(std::chrono::duration<double>(Clock::now() - asked).count(), 1.0);
  EXPECT_FALSE(throttled.groups().empty());
}

TEST_F(ThrottledStreamTest, SettingsChangeAtRuntime) {
  start({ });
  std::mutex mutex;
  std::optional<bool> enabled;
  std::optional<double> rate;
  const rclcpp::QoS state_qos = rclcpp::QoS(1).reliable().transient_local();
  auto enabled_sub = client_->create_subscription<std_msgs::msg::Bool>(
    "/r3v_voxelizer/throttle/get_enabled", state_qos,
    [&](std_msgs::msg::Bool::ConstSharedPtr msg) {
      std::lock_guard<std::mutex> lock(mutex);
      enabled = msg->data;
    });
  auto rate_sub = client_->create_subscription<std_msgs::msg::Float64>(
    "/r3v_voxelizer/throttle/get_rate", state_qos,
    [&](std_msgs::msg::Float64::ConstSharedPtr msg) {
      std::lock_guard<std::mutex> lock(mutex);
      rate = msg->data;
    });
  const auto state_is = [&](bool want_enabled, double want_rate) {
      std::lock_guard<std::mutex> lock(mutex);
      return enabled == want_enabled && rate == want_rate;
    };
  // Off at 1 Hz unless the parameters say otherwise.
  ASSERT_TRUE(waitFor([&]() { return state_is(false, 1.0); }, std::chrono::seconds(10)));

  auto enabled_pub =
    client_->create_publisher<std_msgs::msg::Bool>("/r3v_voxelizer/throttle/set_enabled", 10);
  auto rate_pub =
    client_->create_publisher<std_msgs::msg::Float64>("/r3v_voxelizer/throttle/set_rate", 10);
  ASSERT_TRUE(
    waitFor(
      [&]() {
        return enabled_pub->get_subscription_count() > 0 &&
        rate_pub->get_subscription_count() > 0;
      }, std::chrono::seconds(10)));
  std_msgs::msg::Bool on;
  on.data = true;
  std_msgs::msg::Float64 value;
  value.data = 3.5;
  enabled_pub->publish(on);
  rate_pub->publish(value);
  EXPECT_TRUE(waitFor([&]() { return state_is(true, 3.5); }, std::chrono::seconds(5)));

  // A rate that is not positive is ignored.
  value.data = -1.0;
  rate_pub->publish(value);
  value.data = 0.0;
  rate_pub->publish(value);
  std::this_thread::sleep_for(std::chrono::milliseconds(500));
  EXPECT_TRUE(state_is(true, 3.5));
}

TEST(ThrottledStreamParameters, RejectsBadValues) {
  const auto options = [](std::vector<rclcpp::Parameter> overrides) {
      overrides.emplace_back("voxelizer.backend", "single_threaded");
      rclcpp::NodeOptions node_options;
      node_options.parameter_overrides(overrides);
      return node_options;
    };
  rclcpp::init(0, nullptr);
  EXPECT_ANY_THROW(VoxelizerNode(options({ rclcpp::Parameter("throttle.rate", 0.0) })));
  EXPECT_ANY_THROW(VoxelizerNode(options({ rclcpp::Parameter("throttle.rate", -2.0) })));
  EXPECT_ANY_THROW(
    VoxelizerNode(
      options({ rclcpp::Parameter("voxel_stream.throttled_topic", "/r3v/voxels_downsampled") })));
  EXPECT_ANY_THROW(
    VoxelizerNode(options({ rclcpp::Parameter("voxel_stream.throttled_topic", "") })));
  rclcpp::shutdown();
}

}  // namespace
}  // namespace r3v_ros
