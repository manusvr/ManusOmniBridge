#pragma once

#include "rclcpp/rclcpp.hpp"
#include "manus_ros2_msgs/msg/manus_glove.hpp"
#include "omnihand_2025_node_msgs/msg/motor_angle.hpp"
#include <Eigen/Dense>
#include "ManusUniversalPose.hpp"

namespace ManusMappingStd {
    double clamp(double v, double min, double max);

    omnihand_2025_node_msgs::msg::MotorAngle map_glove_to_angles_direct(
        const manus_ros2_msgs::msg::ManusGlove::SharedPtr& msg,
        bool debug = false,
        rclcpp::Logger logger = rclcpp::get_logger("ManusMappingStd"));

    omnihand_2025_node_msgs::msg::MotorAngle map_glove_to_angles_retargeted(
        const manus_ros2_msgs::msg::ManusGlove::SharedPtr& msg,
        Eigen::VectorXd& joint_angles,
        Eigen::MatrixXf& out_keypoints,
        std::vector<ManusNode>& out_nodes,
        bool debug = false,
        rclcpp::Logger logger = rclcpp::get_logger("ManusMappingStd"));
}