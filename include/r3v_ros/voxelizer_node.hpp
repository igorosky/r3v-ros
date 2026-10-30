#pragma once

#include <chrono>
#include <cstdint>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

#include "Eigen/Geometry"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "std_msgs/msg/empty.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/u_int32.hpp"
#if __has_include("tf2_ros/buffer.hpp")
#include "tf2_ros/buffer.hpp"
#else
#include "tf2_ros/buffer.h"
#endif

#include "r3v_ros/cloud_processor.hpp"
#include "r3v_ros/msg/full_sync_request.hpp"
#include "r3v_ros/msg/voxel_map_update.hpp"
#include "r3v_ros/node_common.hpp"
#include "r3v_ros/voxel_stream.hpp"
#include "r3v_ros/voxelizer_handles.hpp"

namespace r3v_ros {

// Builds a voxel map from any number of point clouds and publishes it on the
// voxel stream (see README.md), which renderer nodes follow.
//
// Needs a multi-threaded executor: point clouds wait for TF, which arrives
// on another callback group.
class VoxelizerNode : public rclcpp::Node
{
public:
  explicit VoxelizerNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
  using Clock = std::chrono::steady_clock;

  // What the voxelizer is made with and point clouds feed it with. Changing
  // it at runtime starts the voxel map over.
  struct VoxelizerSettings {
    double resolution;
    uint32_t destroy_threshold;
    uint32_t construct_threshold;
    uint32_t max_value;
    std::vector<rv::Multipliers> multipliers;  // of every point cloud, as cloud_configs_
  };

  void declareParameters();
  // A new, empty voxelizer with settings_ and the voxelizer.* parameters.
  VoxelizerPtr makeVoxelizer() const;
  void createInterfaces();
  void createSettingInterfaces(const rclcpp::SubscriptionOptions& options);

  // `index` is the cloud's in cloud_configs_.
  void onCloud(size_t index, const sensor_msgs::msg::PointCloud2& cloud);
  void onTick();
  // Applies what the control topics queued since the last tick.
  void applyControls();

  // Transform of `frame` into the fixed frame.
  std::optional<Eigen::Isometry3d> lookup(
    const std::string& frame, const rclcpp::Time& stamp, std::string* error) const;
  // Starts the map over with a new voxelizer, which renderers learn from a
  // FULL message.
  void clearVoxels();
  // Clears the voxel map unless `settings` are the current ones. Returns
  // whether they were not.
  bool setVoxelizerSettings(const VoxelizerSettings& settings);
  void publishSettings();

  bool hasSubscribers() const;
  void publishFull();
  void publishDelta();
  // Publishes `bytes` from `data` as the next message of the stream and
  // returns the size of its (compressed) data.
  size_t publishUpdate(uint8_t kind, const void* data, size_t bytes);

  // Parameters.
  std::string fixed_frame_;
  std::string tf_topic_;
  std::string tf_static_topic_;
  double tf_timeout_;
  std::vector<CloudConfig> cloud_configs_;
  rv::Multipliers multipliers_;  // defaults of every point cloud
  std::string voxelizer_backend_;
  int64_t voxelizer_threads_;
  int64_t voxelizer_buckets_;
  std::string stream_topic_;
  size_t stream_depth_;
  double stream_rate_;
  uint8_t compression_;  // a VoxelMapUpdate::COMPRESSION_* value
  int compression_level_;
  double heartbeat_period_;

  // Declared first so they outlive every callback using them.
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;

  // Voxel callback group only (point clouds and onTick()).
  VoxelizerSettings settings_;
  VoxelizerPtr voxelizer_;  // NULL when making it failed, tried again every tick
  std::vector<rv::PointF> points_;
  // Masks of the blocks changed since the last message, the last one wins.
  std::unordered_map<VoxelKey, uint32_t, VoxelKeyHash> pending_;
  bool full_pending_ = true;  // the next message is a FULL one
  uint64_t session_;
  uint64_t sequence_ = 0;  // of the next message
  std::optional<uint64_t> last_full_;  // sequence of the last FULL message
  Clock::time_point last_publish_;

  // Changes from the control topics, waiting for the next tick. Settings are
  // changed field by field, so that changes arriving together all apply.
  struct MultipliersChange {
    std::optional<float> intensity;
    std::optional<uint32_t> hit;
    std::optional<uint32_t> target;
  };
  struct Controls {
    bool clear_voxels = false;
    std::optional<double> resolution;
    std::optional<uint32_t> destroy_threshold;
    std::optional<uint32_t> construct_threshold;
    std::optional<uint32_t> max_value;
    std::unordered_map<size_t, MultipliersChange> multipliers;  // by cloud index
    std::vector<msg::FullSyncRequest> sync_requests;
  };
  std::mutex controls_mutex_;
  Controls controls_;

  rclcpp::CallbackGroup::SharedPtr tf_group_;
  rclcpp::CallbackGroup::SharedPtr voxel_group_;
  rclcpp::CallbackGroup::SharedPtr control_group_;
  TfSubscriptions tf_subs_;
  std::vector<rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr> cloud_subs_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr clear_voxels_sub_;
  rclcpp::Subscription<msg::FullSyncRequest>::SharedPtr sync_request_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr resolution_sub_;
  rclcpp::Subscription<std_msgs::msg::UInt32>::SharedPtr destroy_threshold_sub_;
  rclcpp::Subscription<std_msgs::msg::UInt32>::SharedPtr construct_threshold_sub_;
  rclcpp::Subscription<std_msgs::msg::UInt32>::SharedPtr max_value_sub_;
  // Current state of the settings above, transient local for late joiners.
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr resolution_pub_;
  rclcpp::Publisher<std_msgs::msg::UInt32>::SharedPtr destroy_threshold_pub_;
  rclcpp::Publisher<std_msgs::msg::UInt32>::SharedPtr construct_threshold_pub_;
  rclcpp::Publisher<std_msgs::msg::UInt32>::SharedPtr max_value_pub_;
  // Multiplier topics of every point cloud, as cloud_configs_.
  struct MultiplierTopics {
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr intensity_sub;
    rclcpp::Subscription<std_msgs::msg::UInt32>::SharedPtr hit_sub;
    rclcpp::Subscription<std_msgs::msg::UInt32>::SharedPtr target_sub;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr intensity_pub;
    rclcpp::Publisher<std_msgs::msg::UInt32>::SharedPtr hit_pub;
    rclcpp::Publisher<std_msgs::msg::UInt32>::SharedPtr target_pub;
  };
  std::vector<MultiplierTopics> multiplier_topics_;
  rclcpp::Publisher<msg::VoxelMapUpdate>::SharedPtr stream_pub_;
  rclcpp::TimerBase::SharedPtr tick_timer_;
};

}  // namespace r3v_ros
