#include "manusOmniPro.hpp"
#include "PinocchioRetargeter.hpp"
#include "ManusUniversalPose.hpp"
#include <cmath>
#include <cstdlib>
#include <filesystem>

namespace ManusMappingPro {

static std::unique_ptr<PinocchioRetargeter> g_retargeter = nullptr;

void initRetargeterO12() {
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
            urdf_path = prefix + "/.." + "/omnihand_pro_description/share/omnihand_pro_description/urdf/omnihand_pro_right.urdf";
        }
    } catch (...) {}
    
    if (!std::filesystem::exists(urdf_path)) {
        urdf_path = "/fixed/path/can/go/here";
    }
    
    g_retargeter = std::make_unique<PinocchioRetargeter>(urdf_path, true);
}

double clamp(double v, double min, double max) {
    return (v < min) ? min : (v > max) ? max : v;
}

std::vector<ManusNode> extractNodes(const manus_ros2_msgs::msg::ManusGlove::SharedPtr& msg) {
    std::vector<ManusNode> nodes;
    nodes.reserve(msg->raw_nodes.size());
    
    for (const auto& node : msg->raw_nodes) {
        ManusNode mNode;
        mNode.id = node.node_id;
        mNode.position = Eigen::Vector3f(
            node.pose.position.x,
            node.pose.position.y,
            node.pose.position.z
        );
        mNode.rotation = Eigen::Quaternionf(
            node.pose.orientation.w,
            node.pose.orientation.x,
            node.pose.orientation.y,
            node.pose.orientation.z
        );
        nodes.push_back(mNode);
    }
    return nodes;
}

omnihand_pro_2025_node_msgs::msg::MotorAngle map_glove_to_angles_direct(
    const manus_ros2_msgs::msg::ManusGlove::SharedPtr& msg,
    bool debug,
    rclcpp::Logger logger)
{
    const double SCALE_RAD = M_PI / 180.0;

    auto out_msg = omnihand_pro_2025_node_msgs::msg::MotorAngle();
    out_msg.angles.assign(12, 0.01);

    const double MIN_SAFE_POS = 0.2;

    for (const auto & ergo : msg->ergonomics) {
        double raw_val = ergo.value * SCALE_RAD;

        // --- THUMB ---
        if (ergo.type == "ThumbMCPSpread")        out_msg.angles[0] = clamp(raw_val, 0.0, 0.94);
        else if (ergo.type == "ThumbMCPStretch")  out_msg.angles[1] = clamp(-raw_val, -1.38, 0.0);
        else if (ergo.type == "ThumbPIPStretch")  out_msg.angles[2] = clamp(-raw_val, -0.82, 0.0);
        else if (ergo.type == "ThumbDIPStretch")  out_msg.angles[3] = clamp(-raw_val, -1.29, 0.0);

        // --- INDEX ---
        else if (ergo.type == "IndexSpread")      out_msg.angles[4] = clamp(raw_val - 0.13, -0.26, 0.26);
        else if (ergo.type == "IndexMCPStretch")   out_msg.angles[5] = clamp(raw_val, 0.0, 1.35);
        else if (ergo.type == "IndexPIPStretch")   out_msg.angles[6] = clamp(raw_val, 0.0, 1.53);

        // --- MIDDLE ---
        else if (ergo.type == "MiddleSpread")     out_msg.angles[7] = clamp(raw_val - 0.13, -0.26, 0.26);
        else if (ergo.type == "MiddleMCPStretch")  out_msg.angles[8] = clamp(raw_val, 0.0, 1.35);
        else if (ergo.type == "MiddlePIPStretch")  out_msg.angles[9] = clamp(raw_val, 0.0, 1.81);

        // --- RING & PINKY ---
        else if (ergo.type == "RingMCPStretch")    out_msg.angles[10] = clamp(raw_val, 0.0, 1.53);
        else if (ergo.type == "PinkyPIPStretch")   out_msg.angles[11] += clamp(raw_val, 0.0, 1.53);
        else if (ergo.type == "PinkyMCPStretch")   out_msg.angles[11] += clamp(raw_val, 0.0, 1.53);

    }

    if (debug) {
        std::stringstream ss;
        ss << "Sending Angles: [";
        for (size_t i = 0; i < out_msg.angles.size(); ++i) {
            ss << std::fixed << std::setprecision(4) << out_msg.angles[i] << (i == 11 ? "" : ", ");
        }
        ss << "]";
        RCLCPP_INFO(logger, "%s", ss.str().c_str());
    }

    return out_msg;
}

omnihand_pro_2025_node_msgs::msg::MotorAngle map_glove_to_angles_retargeted(
    const manus_ros2_msgs::msg::ManusGlove::SharedPtr& msg,
    bool debug,
    rclcpp::Logger logger) 
{
    if (!g_retargeter) {
        initRetargeterO12();
    }

    auto nodes = extractNodes(msg);
    
    Eigen::VectorXd motor_angles = g_retargeter->retarget(nodes);
    
    auto out_msg = omnihand_pro_2025_node_msgs::msg::MotorAngle();
    out_msg.angles.assign(12, 0.0);
    
    for (int i = 0; i < std::min(12, static_cast<int>(motor_angles.size())); ++i) {
        out_msg.angles[i] = motor_angles(i);
    }

    if (debug) {
        std::stringstream ss;
        ss << "Retargeted Angles: [";
        for (size_t i = 0; i < out_msg.angles.size(); ++i) {
            ss << std::fixed << std::setprecision(4) << out_msg.angles[i] << (i == 11 ? "" : ", ");
        }
        ss << "]";
        RCLCPP_INFO(logger, "%s", ss.str().c_str());
    }

    return out_msg;
}

} // namespace ManusMappingPro
