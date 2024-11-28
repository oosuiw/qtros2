#ifndef ROS2NODE_HPP
#define ROS2NODE_HPP

#include "rclcpp/rclcpp.hpp"
#include "autoware_auto_vehicle_msgs/msg/control_mode_report.hpp"
#include "autoware_adapi_v1_msgs/msg/operation_mode_state.hpp"
#include "autoware_adapi_v1_msgs/srv/change_operation_mode.hpp"
#include "std_msgs/msg/float64.hpp"

class Ros2Node : public rclcpp::Node {
public:
    Ros2Node();
    void operation_mode_req_on();
    void operation_mode_req_off();
    void publish_lateral_offset(double offset);

private:
    rclcpp::Publisher<autoware_auto_vehicle_msgs::msg::ControlModeReport>::SharedPtr publisher_control_mode_; 
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr publisher_lateral_offset_;

    rclcpp::TimerBase::SharedPtr timer_;
    
    rclcpp::Client<autoware_adapi_v1_msgs::srv::ChangeOperationMode>::SharedPtr auto_mode_client;
    rclcpp::Client<autoware_adapi_v1_msgs::srv::ChangeOperationMode>::SharedPtr stop_mode_client;
    
    void control_mode_timer_callback();
};

#endif // ROS2NODE_HPP
