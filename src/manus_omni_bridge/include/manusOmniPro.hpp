#pragma once

#include "rclcpp/rclcpp.hpp"
#include "manus_ros2_msgs/msg/manus_glove.hpp"
#include "omnihand_pro_2025_node_msgs/msg/motor_angle.hpp" // Note the PRO msg

namespace ManusMappingPro {
    double clamp(double v, double min, double max);

    omnihand_pro_2025_node_msgs::msg::MotorAngle map_glove_to_angles_direct(
        const manus_ros2_msgs::msg::ManusGlove::SharedPtr& msg,
        bool debug = false,
        rclcpp::Logger logger = rclcpp::get_logger("ManusMappingPro"));

    omnihand_pro_2025_node_msgs::msg::MotorAngle map_glove_to_angles_retargeted(
        const manus_ros2_msgs::msg::ManusGlove::SharedPtr& msg,
        bool debug = false,
        rclcpp::Logger logger = rclcpp::get_logger("ManusMappingPro"));
}