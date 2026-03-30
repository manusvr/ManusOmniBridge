#include "manusOmni.hpp"
#include "PinocchioRetargeter.hpp"
#include "ManusUniversalPose.hpp"
#include <cmath>
#include <cstdlib>
#include <filesystem>

namespace ManusMappingStd {

static std::unique_ptr<PinocchioRetargeter> g_retargeter = nullptr;

void initRetargeterO10() {
    if (g_retargeter) return;
    
    std::string urdf_path;
    try {
        const char* ws = std::getenv("AMENT_PREFIX_PATH");
        if (ws) {
            std::string prefix(ws);
            size_t pos = prefix.find(":");
            if (pos != std::string::npos) {
                prefix = prefix.substr(0, pos);
            }
            urdf_path = prefix + "/.." + "/omnihand_description/share/omnihand_description/urdf/omnihand_right.urdf";
        }
    } catch (...) {}
    
    if (!std::filesystem::exists(urdf_path)) {
        urdf_path = "/hard/path/to/urdf/can/go/here";
    }
    
    g_retargeter = std::make_unique<PinocchioRetargeter>(urdf_path, false);
}

std::vector<ManusNode> extractNodes(const manus_ros2_msgs::msg::ManusGlove::SharedPtr& msg) {
    std::vector<ManusNode> nodes;
    nodes.reserve(msg->raw_nodes.size());
    
    // --- COORDINATE ALIGNMENT ---
    Eigen::AngleAxisf alignment_rotation(M_PI / 2.0, Eigen::Vector3f::UnitZ());

    Eigen::Vector3f raw_wrist(
        msg->raw_nodes[0].pose.position.x,
        msg->raw_nodes[0].pose.position.y,
        msg->raw_nodes[0].pose.position.z
    );

    for (const auto& node : msg->raw_nodes) {
        ManusNode mNode;
        mNode.id = node.node_id;
        
        Eigen::Vector3f raw_pos(
            node.pose.position.x,
            node.pose.position.y,
            node.pose.position.z
        );
        Eigen::Quaternionf raw_rot(
            node.pose.orientation.w,
            node.pose.orientation.x,
            node.pose.orientation.y,
            node.pose.orientation.z
        );

        raw_pos = raw_pos - raw_wrist;
        
        mNode.position = alignment_rotation * raw_pos;
        mNode.position.y() *= -1.0;
        mNode.rotation = alignment_rotation * raw_rot;

        nodes.push_back(mNode);
    }
    return nodes;
}

Eigen::VectorXd jointAnglesToMotorAnglesO10(const Eigen::VectorXd& joint_angles) {
    Eigen::VectorXd motor_angles(10);
    motor_angles.setZero();

    if (joint_angles.size() >= 16) {
        // THUMB (3 Motors)
        motor_angles(0) = joint_angles(11); // Thumb Roll
        motor_angles(1) = joint_angles(12); // Thumb Spread (Abad)
        motor_angles(2) = joint_angles(13); // Thumb MCP Flexion
        
        // INDEX (2 Motors)
        motor_angles(3) = joint_angles(0);  // Index Spread (Abad)
        motor_angles(4) = joint_angles(1);  // Index PIP Flexion
        
        // MIDDLE (1 Motor)
        motor_angles(5) = joint_angles(3);  // Middle PIP Flexion
        
        // RING (2 Motors)
        motor_angles(6) = joint_angles(8);  // Ring Spread (Abad)
        motor_angles(7) = joint_angles(9);  // Ring PIP Flexion
        
        // PINKY (2 Motors)
        motor_angles(8) = joint_angles(5);  // Pinky Spread (Abad)
        motor_angles(9) = joint_angles(6);  // Pinky PIP Flexion
    }
    else {
        for(int i = 0; i < std::min(10, (int)joint_angles.size()); i++) {
            motor_angles(i) = joint_angles(i);
        }
    }
    return motor_angles;
}

double clamp(double v, double min, double max) {
    return (v < min) ? min : (v > max) ? max : v;
}

omnihand_2025_node_msgs::msg::MotorAngle map_glove_to_angles_direct(
    const manus_ros2_msgs::msg::ManusGlove::SharedPtr& msg,
    bool debug,
    rclcpp::Logger logger)
{
    auto out_msg = omnihand_2025_node_msgs::msg::MotorAngle();
    out_msg.angles.assign(10, 0.0);

    const double MIN_SAFE_POS = 0.2;
    const double SCALE_RAD = M_PI / 180.0;

    for (const auto & ergo : msg->ergonomics) {
        double raw_val = ergo.value * SCALE_RAD;

        // --- THUMB ---
        if (ergo.type == "ThumbMCPStretch")        out_msg.angles[0] = clamp(-raw_val*2, -0.03, 1.12);
       
        else if (ergo.type == "ThumbMCPSpread")  out_msg.angles[1] = clamp(-raw_val*2 ,-1.64, 0.05);

        else if (ergo.type == "ThumbDIPStretch")  out_msg.angles[2] += clamp(raw_val, 0, 0.84);
        else if (ergo.type == "ThumbPIPStretch")  out_msg.angles[2] += clamp(raw_val, 0, 0.84);

        // --- INDEX ---
        else if (ergo.type == "IndexSpread")      out_msg.angles[3] = clamp(-raw_val/6, -0.16, 0);
        else if (ergo.type == "IndexPIPStretch")   out_msg.angles[4] += clamp(raw_val, 0.0, 1.48); 
        else if (ergo.type == "IndexMCPStretch")   out_msg.angles[4] += clamp(raw_val, 0.0, 1.48);          

        // --- MIDDLE ---
        else if (ergo.type == "MiddlePIPStretch")  out_msg.angles[5] += clamp(raw_val, 0.0, 1.48);
        else if (ergo.type == "MiddleMCPStretch")  out_msg.angles[5] += clamp(raw_val, 0.0, 1.48);
        
        // --- RING & PINKY ---
        else if (ergo.type == "RingSpread")    out_msg.angles[6] = clamp(-raw_val/5, 0.0, 0.17);
        else if (ergo.type == "RingPIPStretch")  out_msg.angles[7] += clamp(raw_val, 0.0, 1.48);
        else if (ergo.type == "RingMCPStretch")  out_msg.angles[7] += clamp(raw_val, 0.0, 1.48);

        else if (ergo.type == "PinkySpread")   out_msg.angles[8] = clamp(-raw_val/3, 0.0, 0.19);
        else if (ergo.type == "PinkyPIPStretch")  out_msg.angles[9] += clamp(raw_val, 0.0, 1.48) /2; 
        else if (ergo.type == "PinkyMCPStretch")  out_msg.angles[9] += clamp(raw_val, 0.0, 1.48) /2;
    }

    if (debug) {
        std::stringstream ss;
        ss << "Sending Angles: [";
        for (size_t i = 0; i < out_msg.angles.size(); ++i) {
            ss << std::fixed << std::setprecision(4) << out_msg.angles[i] << (i == 9 ? "" : ", ");
        }
        ss << "]";
        RCLCPP_INFO(logger, "%s", ss.str().c_str());
    }

    return out_msg;
}

omnihand_2025_node_msgs::msg::MotorAngle map_glove_to_angles_retargeted(
    const manus_ros2_msgs::msg::ManusGlove::SharedPtr& msg,
    Eigen::VectorXd& joint_angles,
    Eigen::MatrixXf& out_keypoints,
    std::vector<ManusNode>& out_nodes,
    bool debug,
    rclcpp::Logger logger)
{
    if (!g_retargeter) {
        initRetargeterO10();
    }

    auto nodes = extractNodes(msg);
    out_nodes = nodes;
    
    joint_angles = g_retargeter->retarget(nodes);
    out_keypoints = g_retargeter->getKeypoints();
    
    auto out_msg = omnihand_2025_node_msgs::msg::MotorAngle();
    out_msg.angles.assign(10, 0.0);
    
    Eigen::VectorXd motor_angles = jointAnglesToMotorAnglesO10(joint_angles);
    
    for (int i = 0; i < std::min(10, static_cast<int>(motor_angles.size())); ++i) {
        out_msg.angles[i] = motor_angles(i);
    }

    if (debug) {
        // 1. Print Optimizer Output (16 URDF Joints)
        std::stringstream ss_joint;
        ss_joint << "1. OPTIMIZER (16 URDF Joints): [";
        for (int i = 0; i < joint_angles.size(); ++i) {
            ss_joint << joint_angles(i) << (i == joint_angles.size()-1 ? "" : ", ");
        }
        ss_joint << "]";
        RCLCPP_INFO(logger, "%s", ss_joint.str().c_str());

        // 2. Print Final Motor Commands (10 Motors)
        std::stringstream ss_motor;
        ss_motor << "2. MOTORS (10 Commands): [";
        for (int i = 0; i < motor_angles.size(); ++i) {
            ss_motor << motor_angles(i) << (i == motor_angles.size()-1 ? "" : ", ");
        }
        ss_motor << "]";
        RCLCPP_INFO(logger, "%s", ss_motor.str().c_str());
        RCLCPP_INFO(logger, "--------------------------------------------------");
    }

    return out_msg;
}

} // namespace ManusMappingStd