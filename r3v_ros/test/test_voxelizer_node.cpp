#include <gtest/gtest.h>

#include <chrono>
#include <cstdint>
#include <future>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "rclcpp/rclcpp.hpp"

#include "r3v_ros/voxel_stream.hpp"
#include "r3v_ros/voxelizer_node.hpp"

// The voxelizer node's two voxel streams, each compressed as its own
// parameters say. Runs the single_threaded backend, no GPU needed.

namespace r3v_ros {
namespace {

using msg::VoxelMapUpdate;

class VoxelizerNodeTest : public ::testing::Test {
protected:
  static void SetUpTestSuite() {
    rclcpp::init(0, nullptr);
  }

  static void TearDownTestSuite() {
    rclcpp::shutdown();
  }

  // Options of a voxelizer publishing on this test's own topics, plus
  // `overrides`.
  static rclcpp::NodeOptions options(std::vector<rclcpp::Parameter> overrides) {
    overrides.emplace_back("voxelizer.backend", "single_threaded");
    overrides.emplace_back("voxel_stream.topic", "/test_voxelizer_node/voxels");
    overrides.emplace_back(
      "voxel_stream.downsampled_topic", "/test_voxelizer_node/voxels_downsampled");
    rclcpp::NodeOptions node_options;
    node_options.parameter_overrides(overrides);
    return node_options;
  }

  struct Compressions {
    uint8_t full;
    uint8_t downsampled;
  };

  // The compression of the first message of either stream of a voxelizer
  // made with `overrides`.
  static Compressions firstMessages(std::vector<rclcpp::Parameter> overrides) {
    auto voxelizer = std::make_shared<VoxelizerNode>(options(std::move(overrides)));
    auto listener = std::make_shared<rclcpp::Node>("test_voxelizer_node_listener");
    std::mutex mutex;
    std::optional<uint8_t> full;
    std::optional<uint8_t> downsampled;
    std::promise<void> both;
    const auto on_message = [&](std::optional<uint8_t>& slot, const VoxelMapUpdate& msg) {
        std::lock_guard<std::mutex> lock(mutex);
        if (slot.has_value()) {
          return;
        }
        slot = msg.compression;
        if (full.has_value() && downsampled.has_value()) {
          both.set_value();
        }
      };
    const rclcpp::QoS qos = rclcpp::QoS(100).reliable();
    auto full_sub = listener->create_subscription<VoxelMapUpdate>(
      "/test_voxelizer_node/voxels", qos,
      [&](VoxelMapUpdate::ConstSharedPtr msg) { on_message(full, *msg); });
    auto downsampled_sub = listener->create_subscription<VoxelMapUpdate>(
      "/test_voxelizer_node/voxels_downsampled", qos,
      [&](VoxelMapUpdate::ConstSharedPtr msg) { on_message(downsampled, *msg); });

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(voxelizer);
    executor.add_node(listener);
    std::shared_future<void> done = both.get_future().share();
    const rclcpp::FutureReturnCode result =
      executor.spin_until_future_complete(done, std::chrono::seconds(20));
    EXPECT_EQ(result, rclcpp::FutureReturnCode::SUCCESS);
    std::lock_guard<std::mutex> lock(mutex);
    return { full.value_or(0xff), downsampled.value_or(0xff) };
  }
};

TEST_F(VoxelizerNodeTest, DownsampledStreamFollowsTheFullOneByDefault) {
  const Compressions compressions =
    firstMessages({ rclcpp::Parameter("voxel_stream.compression", "zstd") });
  EXPECT_EQ(compressions.full, VoxelMapUpdate::COMPRESSION_ZSTD);
  EXPECT_EQ(compressions.downsampled, VoxelMapUpdate::COMPRESSION_ZSTD);
}

TEST_F(VoxelizerNodeTest, StreamsCompressApart) {
  std::vector<rclcpp::Parameter> overrides;
  overrides.emplace_back("voxel_stream.compression", "none");
  overrides.emplace_back("voxel_stream.downsampled_compression", "zstd");
  overrides.emplace_back("voxel_stream.downsampled_compression_level", 3);
  const Compressions compressions = firstMessages(std::move(overrides));
  EXPECT_EQ(compressions.full, VoxelMapUpdate::COMPRESSION_NONE);
  EXPECT_EQ(compressions.downsampled, VoxelMapUpdate::COMPRESSION_ZSTD);
}

TEST_F(VoxelizerNodeTest, RejectsUnknownCompressions) {
  EXPECT_ANY_THROW(
    VoxelizerNode(options({ rclcpp::Parameter("voxel_stream.downsampled_compression", "lz4") })));
  EXPECT_ANY_THROW(
    VoxelizerNode(
      options({ rclcpp::Parameter("voxel_stream.downsampled_compression_level", 1000) })));
}

}  // namespace
}  // namespace r3v_ros
