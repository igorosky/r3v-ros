#include "r3v_ros/node_common.hpp"

#include <cmath>
#include <limits>
#include <stdexcept>

#include "tf2_ros/qos.hpp"

namespace r3v_ros {

void require(bool condition, const std::string& message) {
  if (!condition) {
    throw std::invalid_argument(message);
  }
}

bool validColor(double r, double g, double b) {
  const auto unit = [](double c) { return c >= 0.0 && c <= 1.0; };
  return unit(r) && unit(g) && unit(b);
}

bool validPositiveFloat(double value) {
  return value > 0.0 && value <= std::numeric_limits<float>::max() &&
         static_cast<float>(value) > 0.0f;
}

bool validFloat(double value) {
  return std::isfinite(value) && std::abs(value) <= std::numeric_limits<float>::max();
}

uint32_t declareU32(rclcpp::Node& node, const std::string& name, int64_t fallback) {
  const int64_t v = node.declare_parameter<int64_t>(name, fallback);
  require(v >= 0 && v <= std::numeric_limits<uint32_t>::max(), name + " must fit in 32 bits");
  return static_cast<uint32_t>(v);
}

TfSubscriptions subscribeTf(
  rclcpp::Node& node, tf2_ros::Buffer& buffer, const std::string& tf_topic,
  const std::string& tf_static_topic, const rclcpp::CallbackGroup::SharedPtr& group) {
  const auto feed = [&node, &buffer](const tf2_msgs::msg::TFMessage& msg, bool is_static) {
      for (const auto& transform : msg.transforms) {
        try {
          buffer.setTransform(transform, node.get_name(), is_static);
        } catch (const tf2::TransformException& e) {
          RCLCPP_WARN_THROTTLE(
            node.get_logger(), *node.get_clock(), 5000, "Bad transform: %s", e.what());
        }
      }
    };
  rclcpp::SubscriptionOptions options;
  options.callback_group = group;
  TfSubscriptions subs;
  subs.tf = node.create_subscription<tf2_msgs::msg::TFMessage>(
    tf_topic, tf2_ros::DynamicListenerQoS(),
    [feed](tf2_msgs::msg::TFMessage::ConstSharedPtr msg) { feed(*msg, false); }, options);
  subs.tf_static = node.create_subscription<tf2_msgs::msg::TFMessage>(
    tf_static_topic, tf2_ros::StaticListenerQoS(),
    [feed](tf2_msgs::msg::TFMessage::ConstSharedPtr msg) { feed(*msg, true); }, options);
  return subs;
}

rclcpp::QoS voxelStreamQos(size_t depth) {
  return rclcpp::QoS(rclcpp::KeepLast(depth)).reliable().durability_volatile();
}

}  // namespace r3v_ros
