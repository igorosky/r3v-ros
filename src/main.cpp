#include <exception>
#include <memory>

#include "rclcpp/rclcpp.hpp"

#include "r3v_ros/r3v_node.hpp"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  int result = 0;
  try {
    auto node = std::make_shared<r3v_ros::R3vNode>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
  } catch (const std::exception& e) {
    RCLCPP_FATAL(rclcpp::get_logger("r3v_node"), "%s", e.what());
    result = 1;
  }
  rclcpp::shutdown();
  return result;
}
