#pragma once

#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include <Eigen/Geometry>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <image_transport/image_transport.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#if __has_include(<tf2_ros/buffer.hpp>)
#include <tf2_ros/buffer.hpp>
#else
#include <tf2_ros/buffer.h>
#endif

#include "r3v_ros/camera.hpp"
#include "r3v_ros/cloud_processor.hpp"
#include "r3v_ros/r3v_handles.hpp"
#include "r3v_ros/robot_scene.hpp"

namespace r3v_ros
{

// Renders the robot from robot_description + TF and the voxel map built
// from any number of point clouds, and publishes it as an image stream.
//
// Needs a multi-threaded executor: point clouds wait for TF, which arrives
// on another callback group.
class R3vNode : public rclcpp::Node
{
public:
  explicit R3vNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void declareParameters();
  void createVoxelizer();
  void createRenderer();
  void createInterfaces();

  void onTf(const tf2_msgs::msg::TFMessage & msg, bool is_static);
  void onCloud(const CloudConfig & config, const sensor_msgs::msg::PointCloud2 & cloud);
  void onRobotDescription(const std::string & urdf_xml);
  void onRender();

  // Transform of `frame` into the fixed frame (latest when `stamp` is 0).
  std::optional<Eigen::Isometry3d> lookup(
    const std::string & frame, const rclcpp::Time & stamp, double timeout,
    std::string * error = nullptr) const;
  bool cameraView(CameraView & view) const;
  void flushVoxels();
  void publish(const std::vector<uint8_t> & bgra, const rclcpp::Time & stamp);

  // Parameters.
  std::string fixed_frame_;
  double rate_;
  std::string tf_topic_;
  std::string tf_static_topic_;
  double tf_timeout_;
  std::string robot_description_topic_;
  std::string robot_description_file_;
  std::vector<std::string> package_paths_;
  bool double_sided_;
  std::string output_topic_;
  uint32_t width_;
  uint32_t height_;
  std::string encoding_;
  std::string output_frame_id_;
  bool lazy_;
  std::string camera_frame_;
  Eigen::Vector3d camera_position_;
  Eigen::Vector3d camera_target_;
  CameraLens lens_;
  std::string camera_pose_topic_;
  std::vector<CloudConfig> cloud_configs_;
  double voxel_size_;

  // Declared first so they outlive every callback using them.
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  VoxelizerPtr voxelizer_;
  RendererPtr renderer_;
  TesselatorPtr tesselator_;
  std::unique_ptr<RobotScene> robot_;

  // Voxel blocks from cloud callbacks, waiting for the next frame.
  std::mutex pending_mutex_;
  std::vector<rr::R3vPointI> pending_blocks_;

  // Render state, only touched from the render callback group.
  std::optional<geometry_msgs::msg::PoseStamped> camera_pose_;
  std::vector<uint8_t> frame_;
  std::optional<rclcpp::Time> rendered_stamp_;  // stamp of the frame the renderer holds
  std::vector<rr::R3vPointI> blocks_;
  std::vector<rv::PointF> points_;  // cloud callback group only

  rclcpp::CallbackGroup::SharedPtr tf_group_;
  rclcpp::CallbackGroup::SharedPtr cloud_group_;
  rclcpp::CallbackGroup::SharedPtr render_group_;
  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr tf_sub_;
  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr tf_static_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr robot_description_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr camera_pose_sub_;
  std::vector<rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr> cloud_subs_;
  image_transport::Publisher image_pub_;
  rclcpp::TimerBase::SharedPtr render_timer_;
};

}  // namespace r3v_ros
