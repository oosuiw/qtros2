#ifndef ROS2NODE_HPP
#define ROS2NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include "autoware_auto_vehicle_msgs/msg/control_mode_report.hpp"
#include "autoware_adapi_v1_msgs/msg/operation_mode_state.hpp"
#include "autoware_adapi_v1_msgs/srv/change_operation_mode.hpp"
#include "std_msgs/msg/float64.hpp"

class Ros2Node : public rclcpp::Node
{
public:
    explicit Ros2Node();

    // 요청 메서드
    void operation_mode_req_on();
    void operation_mode_req_off();
    void publish_lateral_offset(const double offset);

    // 퍼블리셔
    rclcpp::Publisher<autoware_auto_vehicle_msgs::msg::ControlModeReport>::SharedPtr publisher_control_mode_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr publisher_lateral_offset_;

    // 클라이언트
    rclcpp::Client<autoware_adapi_v1_msgs::srv::ChangeOperationMode>::SharedPtr auto_mode_client_;
    rclcpp::Client<autoware_adapi_v1_msgs::srv::ChangeOperationMode>::SharedPtr stop_mode_client_;

    // 타이머
    rclcpp::TimerBase::SharedPtr timer_;

    // 상태 값
    struct OperationModeStatus
    {
        bool is_auto_mode_on{false};
        bool is_stop_mode_on{false};
    };

    OperationModeStatus operation_mode_status_;

    // 콜백 함수
    void control_mode_timer_callback();
};

#endif // ROS2NODE_HPP
