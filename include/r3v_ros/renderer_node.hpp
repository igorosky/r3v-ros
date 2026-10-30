#pragma once

#include <cstdint>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include "Eigen/Geometry"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "image_transport/image_transport.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/color_rgba.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/u_int32.hpp"
#if __has_include("tf2_ros/buffer.hpp")
#include "tf2_ros/buffer.hpp"
#else
#include "tf2_ros/buffer.h"
#endif

#include "r3v_ros/camera.hpp"
#include "r3v_ros/msg/full_sync_request.hpp"
#include "r3v_ros/msg/voxel_map_update.hpp"
#include "r3v_ros/node_common.hpp"
#include "r3v_ros/renderer_handles.hpp"
#include "r3v_ros/robot_scene.hpp"
#include "r3v_ros/voxel_map.hpp"
#include "r3v_ros/voxel_stream.hpp"

namespace r3v_ros {

// Renders the robot from robot_description + TF and the voxel map of the
// voxel stream (see README.md) a voxelizer node publishes, and publishes it
// as an image stream.
//
// Wants a multi-threaded executor: the voxel stream is decoded on its own
// callback group while frames render.
class RendererNode : public rclcpp::Node
{
public:
  explicit RendererNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
  // How voxels are colored: by distance from the camera through turbo, or
  // by height through a hypsometric colormap (see colormap in
  // config/r3v.yaml).
  struct Colormap {
    double depth_range;  // meters the distance colormap runs over once
    double color_offset;  // meters added to the distance
    bool by_height;
    bool height_relative;  // neutral height at height_frame_ instead of 0
    double height_offset;  // meters above the neutral height
    double height_scale;  // meters, the knee of the height colormap
    uint32_t height_bands;  // 0 is continuous
  };

  void declareParameters();
  void createRenderer();
  // Applies the frame size, background color and colormap.
  void configureRenderer();
  // Applies the voxel size and marching cubes.
  void configureTesselator();
  void createInterfaces();
  // The ~/colormap/set_* subscriptions and get_* publishers.
  void createColormapInterfaces(const rclcpp::SubscriptionOptions& options);
  // The subscriptions a disabled renderer drops: TF, the voxel stream and
  // robot_description.
  void subscribeInputs();
  void unsubscribeInputs();

  void onVoxelUpdate(const msg::VoxelMapUpdate& msg);
  void onSyncRetry();
  void onRobotDescription(const std::string& urdf_xml);
  void onHiddenLinks(const std::string& links);
  void onRender();
  // Applies what the control topics queued since the last frame.
  void applyControls();

  // Asks the voxelizer for a FULL message. Needs stream_mutex_.
  void requestFullSync();
  bool cameraView(CameraView& view) const;
  // Hands what the voxel stream queued since the last frame to the
  // tesselator.
  void flushVoxels();
  void tesselate(const std::vector<WireBlock>& blocks);
  // Replaces every voxel the tesselator holds with `groups` and uploads it.
  void replaceTesselated(const std::vector<WireGroup>& groups);
  // Uploads the chunks the tesselator changed since the last call.
  void uploadGeometryDelta();
  void setEnabled(bool enabled);
  void setVoxelsVisible(bool visible);
  void setMarchingCubes(bool enabled);
  void setBackgroundColor(const rr::R3vVec3& color);
  void setColormap(const Colormap& colormap);
  // Height coloring relative to height_frame_: follows that frame's height.
  void updateHeightReference();
  void publishEnabled();
  void publishMarchingCubes();
  void publishBackgroundColor();
  void publishColormap();
  void publish(const std::vector<uint8_t>& bgra, const rclcpp::Time& stamp);

  // Parameters.
  std::string fixed_frame_;
  double rate_;
  std::string tf_topic_;
  std::string tf_static_topic_;
  std::string robot_description_topic_;
  std::string robot_description_file_;
  std::vector<std::string> package_paths_;
  bool double_sided_;
  std::vector<std::string> hidden_links_;
  std::string output_topic_;
  uint32_t width_;
  uint32_t height_;
  std::string encoding_;
  std::string output_frame_id_;
  bool lazy_;
  std::string output_qos_;
  std::string camera_frame_;
  Eigen::Vector3d camera_position_;
  Eigen::Vector3d camera_target_;
  CameraLens lens_;
  std::string camera_pose_topic_;
  std::string height_frame_;
  double height_gain_;
  std::string stream_topic_;
  size_t stream_depth_;
  std::string request_topic_;
  double sync_retry_;

  // Declared first so they outlive every callback using them.
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  RendererPtr renderer_;
  TesselatorPtr tesselator_;
  std::unique_ptr<RobotScene> robot_;

  // Where the voxel stream stands, under stream_mutex_. Changed by the
  // stream callback group and by setEnabled().
  struct StreamSync {
    std::optional<uint64_t> session;  // none before the first message
    uint64_t last = 0;  // sequence of the last message
    uint64_t since = 0;  // sequence of the first message in a row
    bool synced = false;  // a FULL message arrived since `since`
    double resolution = 0.0;  // of that FULL message
  };
  // What the voxel stream decoded for the next frame.
  struct PendingVoxels {
    std::optional<std::vector<WireGroup>> full;  // replaces the map, before `blocks`
    double resolution = 0.0;  // of `full`
    std::vector<WireBlock> blocks;
  };
  std::mutex stream_mutex_;
  bool listening_ = false;  // enabled, takes voxel stream messages
  uint64_t epoch_ = 0;  // bumped by setEnabled(), drops messages decoded before
  StreamSync sync_;
  PendingVoxels pending_;

  // Changes from the control topics, waiting for the next frame.
  struct Controls {
    std::optional<bool> enabled;
    std::optional<geometry_msgs::msg::PoseStamped> camera_pose;
    std::optional<std::vector<std::string>> hidden_links;
    std::optional<bool> voxels_visible;
    std::optional<bool> marching_cubes;
    std::optional<rr::R3vVec3> background_color;
    std::optional<double> color_depth_range;
    std::optional<double> color_offset;
    std::optional<bool> color_by_height;
    std::optional<bool> height_relative;
    std::optional<double> height_offset;
    std::optional<double> height_scale;
    std::optional<uint32_t> height_bands;
  };
  std::mutex controls_mutex_;
  Controls controls_;

  // Render state, only touched from the render callback group.
  bool enabled_;
  std::optional<geometry_msgs::msg::PoseStamped> camera_pose_;
  std::vector<uint8_t> frame_;
  std::optional<rclcpp::Time> rendered_stamp_;  // stamp of the frame the renderer holds
  // The map of the voxel stream, so that it can be hidden and shown without
  // the voxelizer's help.
  VoxelMap map_;
  double resolution_;  // voxel size the tesselator is configured with
  std::string loaded_urdf_;  // robot description the robot was loaded from
  bool voxels_visible_;
  bool marching_cubes_;
  rr::R3vVec3 background_color_;  // sRGB, each component in 0..1
  Colormap colormap_;
  double height_reference_ = 0.0;  // height_frame_ z in fixed_frame_ the renderer got

  rclcpp::CallbackGroup::SharedPtr tf_group_;
  rclcpp::CallbackGroup::SharedPtr stream_group_;
  rclcpp::CallbackGroup::SharedPtr render_group_;
  rclcpp::CallbackGroup::SharedPtr control_group_;
  TfSubscriptions tf_subs_;
  rclcpp::Subscription<msg::VoxelMapUpdate>::SharedPtr stream_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr robot_description_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr enabled_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr camera_pose_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr hidden_links_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr voxels_visible_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr marching_cubes_sub_;
  rclcpp::Subscription<std_msgs::msg::ColorRGBA>::SharedPtr background_color_sub_;
  std::vector<rclcpp::SubscriptionBase::SharedPtr> colormap_subs_;
  // Current state of the settings above, transient local for late joiners.
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr enabled_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr marching_cubes_pub_;
  rclcpp::Publisher<std_msgs::msg::ColorRGBA>::SharedPtr background_color_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr color_depth_range_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr color_offset_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr color_mode_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr height_relative_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr height_offset_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr height_scale_pub_;
  rclcpp::Publisher<std_msgs::msg::UInt32>::SharedPtr height_bands_pub_;
  rclcpp::Publisher<msg::FullSyncRequest>::SharedPtr request_pub_;
  image_transport::Publisher image_pub_;
  rclcpp::TimerBase::SharedPtr render_timer_;
  rclcpp::TimerBase::SharedPtr sync_timer_;
};

}  // namespace r3v_ros
