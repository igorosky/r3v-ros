#pragma once

#include <cstddef>
#include <cstdint>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "tf2_msgs/msg/tf_message.hpp"
#if __has_include("tf2_ros/buffer.hpp")
#include "tf2_ros/buffer.hpp"
#else
#include "tf2_ros/buffer.h"
#endif

namespace r3v_ros {

// Throws std::invalid_argument with `message` unless `condition`.
void require(bool condition, const std::string& message);

// sRGB components between 0 and 1.
bool validColor(double r, double g, double b);

// For values the libraries take as a positive float.
bool validPositiveFloat(double value);

bool validFloat(double value);

// Declares the integer parameter `name` of `node`, which has to fit in 32
// bits.
uint32_t declareU32(rclcpp::Node& node, const std::string& name, int64_t fallback);

struct TfSubscriptions {
  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr tf;
  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr tf_static;
};

// Feeds `buffer` from `tf_topic` and `tf_static_topic` in `group`. `node`
// and `buffer` have to outlive the subscriptions.
TfSubscriptions subscribeTf(
  rclcpp::Node& node, tf2_ros::Buffer& buffer, const std::string& tf_topic,
  const std::string& tf_static_topic, const rclcpp::CallbackGroup::SharedPtr& group);

// QoS of the voxel stream: reliable and in order, `depth` messages kept for
// slow subscribers (losing more shows up as a gap in the sequence).
rclcpp::QoS voxelStreamQos(size_t depth);

}  // namespace r3v_ros
