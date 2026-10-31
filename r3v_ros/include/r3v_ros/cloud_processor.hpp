#pragma once

#include <string>
#include <vector>

#include "Eigen/Geometry"
#include "sensor_msgs/msg/point_cloud2.hpp"

#include "r3v_ros/voxelizer_handles.hpp"

namespace r3v_ros {

struct CloudConfig {
  std::string name;
  std::string topic;
  std::string qos;           // "sensor_data" or "reliable"
  std::string sensor_frame;  // ray origin; empty = the cloud's frame
  double min_range = 0.0;
  double max_range = 30.0;
  int decimation = 1;        // use every n-th point
  std::string intensity_field;  // empty = constant intensity_scale
  double intensity_scale = 1.0;
};

// Reads the points of `cloud`, keeps finite ones between min_range and
// max_range from the sensor, and transforms them into the renderer world.
// `cloud_to_fixed` maps the cloud frame to the fixed frame, where
// `sensor_origin` lies. Throws std::runtime_error for a malformed or
// unsupported cloud.
void extractPoints(
  const sensor_msgs::msg::PointCloud2& cloud, const CloudConfig& config,
  const Eigen::Isometry3d& cloud_to_fixed, const Eigen::Vector3d& sensor_origin,
  std::vector<rv::PointF>& out);

}  // namespace r3v_ros
