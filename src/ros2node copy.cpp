#include "qtros2/ros2node.hpp"
#include "std_msgs/msg/bool.hpp"
#include "autoware_auto_vehicle_msgs/msg/control_mode_report.hpp"

Ros2Node::Ros2Node()
  : rclcpp::Node("ros2_node")
{
  publisher_bool_ = this->create_publisher<std_msgs::msg::Bool>("/control_mode", 10);
  publisher_control_mode_ = this->create_publisher<autoware_auto_vehicle_msgs::msg::ControlModeReport>("/vehicle/status/control_mode", 10);
}

void Ros2Node::publish_message(bool message)
{
  std_msgs::msg::Bool ros2_message;
  ros2_message.data = message;  
  publisher_bool_->publish(ros2_message);  
}

void Ros2Node::publish_control_mode(const autoware_auto_vehicle_msgs::msg::ControlModeReport& control_mode_msg) {
    publisher_control_mode_->publish(control_mode_msg); 
}
