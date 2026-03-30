#include "rclcpp/rclcpp.hpp"

#include <sensor_msgs/msg/joint_state.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "manus_ros2_msgs/msg/manus_glove.hpp"
#include "omnihand_2025_node_msgs/msg/motor_angle.hpp"
#include "omnihand_pro_2025_node_msgs/msg/motor_angle.hpp"

// Include your specific logic files
#include "manusOmni.hpp"
#include "manusOmniPro.hpp"
#include "ManusUniversalPose.hpp"

class ManusToOmniBridge : public rclcpp::Node {
public:
    ManusToOmniBridge() : Node("manus_to_omni_bridge") {
        this->declare_parameter("is_pro", false);
        is_pro_ = this->get_parameter("is_pro").as_bool();

        this->declare_parameter("manus_retargeting", true);
        manus_retargeting_ = this->get_parameter("manus_retargeting").as_bool();

        this->declare_parameter("debug_print", true);
        debug_print_ = this->get_parameter("debug_print").as_bool();

        this->declare_parameter("hand_type", "right");
        std::string hand_type = this->get_parameter("hand_type").as_string();

        joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
        marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("manus_markers", 10);

        subscription_ = this->create_subscription<manus_ros2_msgs::msg::ManusGlove>(
            "manus_glove_0", 10, 
            std::bind(&ManusToOmniBridge::topic_callback, this, std::placeholders::_1));

        if (is_pro_) {
            pro_publisher_ = this->create_publisher<omnihand_pro_2025_node_msgs::msg::MotorAngle>(
                "/omnihand/omnihand_pro_2025/" + hand_type + "/motor_angle_cmd", 10);
            RCLCPP_INFO(this->get_logger(), "Manus to Omnihand PRO Bridge Started");
        } else {
            std_publisher_ = this->create_publisher<omnihand_2025_node_msgs::msg::MotorAngle>(
                "/omnihand/omnihand_2025/" + hand_type + "/motor_angle_cmd", 10);
            RCLCPP_INFO(this->get_logger(), "Manus to Omnihand STD Bridge Started");
        }
    }

private:
    void topic_callback(const manus_ros2_msgs::msg::ManusGlove::SharedPtr msg) const {
        Eigen::VectorXd joint_angles;
        Eigen::MatrixXf target_keypoints;
        std::vector<ManusNode> nodes;
        if (is_pro_) {
            auto out_msg = omnihand_pro_2025_node_msgs::msg::MotorAngle();
            if (manus_retargeting_) {
                out_msg = ManusMappingPro::map_glove_to_angles_retargeted(msg, debug_print_, this->get_logger());
            } else {
                out_msg = ManusMappingPro::map_glove_to_angles_direct(msg, debug_print_, this->get_logger());
            }
            out_msg.header.stamp = this->now();
            pro_publisher_->publish(out_msg);
        } else {
            auto out_msg = omnihand_2025_node_msgs::msg::MotorAngle();
            if (manus_retargeting_) {
                out_msg = ManusMappingStd::map_glove_to_angles_retargeted(msg, joint_angles, target_keypoints, nodes, debug_print_, this->get_logger());
            } else {
                out_msg = ManusMappingStd::map_glove_to_angles_direct(msg, debug_print_, this->get_logger());
            }
            out_msg.header.stamp = this->now();
            std_publisher_->publish(out_msg);
            if(rviz_){
                if (joint_angles.size() > 0)
                {
                    publish_visualizations(nodes, joint_angles, target_keypoints);
                }
            }
        }
    }

    void publish_visualizations(
        const std::vector<ManusNode>& nodes, 
        const Eigen::VectorXd& joint_angles,
        const Eigen::MatrixXf& target_keypoints) const
    {
        auto now = this->now();

        // ---------------------------------------------------------
        // 1. PUBLISH ROBOT JOINT STATES (Animates the URDF in RViz)
        // ---------------------------------------------------------
        auto joint_msg = sensor_msgs::msg::JointState();
        joint_msg.header.stamp = now;
        
        // The fixed frame of your robot URDF
        joint_msg.header.frame_id = "base_link"; 
        // IMPORTANT: These names must exactly match the URDF!
        std::vector<std::string> joint_names = {
            "R_index_abad_joint", "R_index_pip_joint", "R_index_dip_joint",
            "R_middle_pip_joint", "R_middle_dip_joint",
            "R_pinky_abad_joint", "R_pinky_pip_joint", "R_pinky_dip_joint",
            "R_ring_abad_joint", "R_ring_pip_joint", "R_ring_dip_joint",
            "R_thumb_roll_joint", "R_thumb_abad_joint", "R_thumb_mcp_joint", "R_thumb_pip_joint", "R_thumb_dip_joint"
        };

        joint_msg.name = joint_names;
        for (int i = 0; i < joint_angles.size() && i < joint_names.size(); ++i) {
            joint_msg.position.push_back(joint_angles(i));
        }
        joint_state_pub_->publish(joint_msg);

        // 2. PUBLISH MANUS GLOVE MARKERS (Green Dots)
        auto marker_array = visualization_msgs::msg::MarkerArray();
        for (size_t i = 0; i < nodes.size(); ++i) { 
            visualization_msgs::msg::Marker marker;
            marker.header.frame_id = "base_link"; 
            marker.header.stamp = now;
            marker.ns = "manus_skeleton";
            marker.id = i;
            marker.type = visualization_msgs::msg::Marker::SPHERE;
            marker.action = visualization_msgs::msg::Marker::ADD;
            
            marker.pose.position.x = nodes[i].position.x();
            marker.pose.position.y = nodes[i].position.y();
            marker.pose.position.z = nodes[i].position.z();
            
            marker.scale.x = 0.01;
            marker.scale.y = 0.01;
            marker.scale.z = 0.01;
            marker.color.a = 1.0; 
            marker.color.r = 0.0;
            marker.color.g = 1.0;
            marker.color.b = 0.0;

            marker_array.markers.push_back(marker);
        }
        marker_pub_->publish(marker_array);

        // 3. PUBLISH TARGET KEYPOINTS (The Optimizer's "Ghost Hand" in RED)
        for (int i = 0; i < target_keypoints.cols(); ++i) {
            visualization_msgs::msg::Marker target_marker;
            target_marker.header.frame_id = "base_link"; 
            target_marker.header.stamp = now;
            target_marker.ns = "optimizer_targets";
            target_marker.id = i;
            target_marker.type = visualization_msgs::msg::Marker::SPHERE;
            target_marker.action = visualization_msgs::msg::Marker::ADD;
            
            target_marker.pose.position.x = target_keypoints(0, i);
            target_marker.pose.position.y = target_keypoints(1, i);
            target_marker.pose.position.z = target_keypoints(2, i);
            
            target_marker.scale.x = 0.012; 
            target_marker.scale.y = 0.012;
            target_marker.scale.z = 0.012;
            
            target_marker.color.a = 0.8; 
            target_marker.color.r = 1.0; 
            target_marker.color.g = 0.0;
            target_marker.color.b = 0.0;

            marker_array.markers.push_back(target_marker);
        }
        marker_pub_->publish(marker_array);
    }

    bool is_pro_;
    bool manus_retargeting_;
    rclcpp::Subscription<manus_ros2_msgs::msg::ManusGlove>::SharedPtr subscription_;
    rclcpp::Publisher<omnihand_2025_node_msgs::msg::MotorAngle>::SharedPtr std_publisher_;
    rclcpp::Publisher<omnihand_pro_2025_node_msgs::msg::MotorAngle>::SharedPtr pro_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
    bool debug_print_ = false;
    bool rviz_ = true;

    const double SCALE_RAD = M_PI / 180.0;
};


int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ManusToOmniBridge>());
    rclcpp::shutdown();
    return 0;
}