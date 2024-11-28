#include "qtros2/ros2node.hpp"

Ros2Node::Ros2Node() : rclcpp::Node("ros2_node")
{
    publisher_control_mode_ = this->create_publisher<autoware_auto_vehicle_msgs::msg::ControlModeReport>("/vehicle/status/control_mode", 10);
    publisher_lateral_offset_ = this->create_publisher<std_msgs::msg::Float64>("/planning/scenario_planning/lane_driving/behavior_planning/behavior_path_planner/input/lateral_shift", 10);


    auto_mode_client = this->create_client<autoware_adapi_v1_msgs::srv::ChangeOperationMode>("/api/operation_mode/change_to_autonomous");
    stop_mode_client = this->create_client<autoware_adapi_v1_msgs::srv::ChangeOperationMode>("/api/operation_mode/change_to_stop");

    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100), 
        std::bind(&Ros2Node::control_mode_timer_callback, this)
    );
}

void Ros2Node::control_mode_timer_callback()
{
    autoware_auto_vehicle_msgs::msg::ControlModeReport control_mode_msg;
    control_mode_msg.mode = 1;
    publisher_control_mode_->publish(control_mode_msg);  
}

void Ros2Node::operation_mode_req_on()
{
  std::shared_ptr<autoware_adapi_v1_msgs::srv::ChangeOperationMode::Request> request = 
    std::make_shared<autoware_adapi_v1_msgs::srv::ChangeOperationMode::Request>();
  std::shared_future<std::shared_ptr<autoware_adapi_v1_msgs::srv::ChangeOperationMode::Response>> result = 
    auto_mode_client->async_send_request(request);
}

void Ros2Node::operation_mode_req_off()
{
  std::shared_ptr<autoware_adapi_v1_msgs::srv::ChangeOperationMode::Request> request = 
    std::make_shared<autoware_adapi_v1_msgs::srv::ChangeOperationMode::Request>();
  std::shared_future<std::shared_ptr<autoware_adapi_v1_msgs::srv::ChangeOperationMode::Response>> result = 
    stop_mode_client->async_send_request(request);
}

void Ros2Node::publish_lateral_offset(double offset)
{
    std_msgs::msg::Float64 msg;
    msg.data = offset;
    publisher_lateral_offset_->publish(msg);
}
