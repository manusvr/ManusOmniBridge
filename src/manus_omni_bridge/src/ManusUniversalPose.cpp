#include "ManusUniversalPose.hpp"
#include <Eigen/Geometry>
#include <cmath>

Eigen::Matrix3f ManusUniversalPoseConverter::computePalmFrame(
    const Eigen::Vector3f& wrist,
    const Eigen::Vector3f& index_mcp,
    const Eigen::Vector3f& pinky_mcp) 
{
    Eigen::Vector3f x_axis = (index_mcp - wrist).normalized();
    Eigen::Vector3f temp_side = (pinky_mcp - index_mcp).normalized();
    
    Eigen::Vector3f y_axis = x_axis.cross(temp_side).normalized();
    Eigen::Vector3f z_axis = x_axis.cross(y_axis).normalized();

    Eigen::Matrix3f frame;
    frame.col(0) = x_axis;
    frame.col(1) = y_axis;
    frame.col(2) = z_axis;
    return frame;
}


UniversalHandPose ManusUniversalPoseConverter::convert(
    const std::vector<ManusNode>& nodes,
    bool is_right_hand) 
{
    UniversalHandPose pose;
    pose.palm_frame = Eigen::Matrix3f::Identity();
    if (nodes.size() < 25) return pose;
    
    std::array<Eigen::Vector3f, 25> global_pos;
    std::array<Eigen::Quaternionf, 25> global_rot;

    global_pos[0] = Eigen::Vector3f::Zero();
    global_rot[0] = Eigen::Quaternionf::Identity();

    // Map of each node to its parent (matches Manus 25-node skeleton)
    int parent[25] = { -1, 0,1,2,3, 0,5,6,7,8, 0,10,11,12,13, 0,15,16,17,18, 0,20,21,22,23 };

    // Calculate actual 3D positions in space via Forward Kinematics
    for (int i = 1; i < 25; ++i) {
        int p = parent[i];
        
        Eigen::Vector3f local_pos = nodes[i].position;
        Eigen::Quaternionf local_rot = nodes[i].rotation;
        local_rot.normalize();

        global_rot[i] = global_rot[p] * local_rot;
        global_pos[i] = global_pos[p] + (global_rot[p] * local_pos);
    }
    
    pose.wrist = global_pos[0]; 
    pose.thumb[0] = global_pos[1]; pose.thumb[1] = global_pos[2]; pose.thumb[2] = global_pos[3]; pose.thumb[3] = global_pos[4]; 
    pose.index[0] = global_pos[6]; pose.index[1] = global_pos[7]; pose.index[2] = global_pos[8]; pose.index[3] = global_pos[9];
    pose.middle[0] = global_pos[11]; pose.middle[1] = global_pos[12]; pose.middle[2] = global_pos[13]; pose.middle[3] = global_pos[14];
    pose.ring[0] = global_pos[16]; pose.ring[1] = global_pos[17]; pose.ring[2] = global_pos[18]; pose.ring[3] = global_pos[19];
    pose.pinky[0] = global_pos[21]; pose.pinky[1] = global_pos[22]; pose.pinky[2] = global_pos[23]; pose.pinky[3] = global_pos[24];

    pose.palm_frame = computePalmFrame(global_pos[0], global_pos[6], global_pos[21]);

    return pose;
}
