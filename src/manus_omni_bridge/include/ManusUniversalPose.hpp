#ifndef MANUS_UNIVERSAL_POSE_HPP
#define MANUS_UNIVERSAL_POSE_HPP

#include <Eigen/Dense>
#include <array>
#include <vector>
#include <cstdint>

constexpr int NUM_KEYPOINTS = 21;

struct UniversalHandPose {
    Eigen::Vector3f wrist;
    std::array<Eigen::Vector3f, 4> thumb;
    std::array<Eigen::Vector3f, 4> index;
    std::array<Eigen::Vector3f, 4> middle;
    std::array<Eigen::Vector3f, 4> ring;
    std::array<Eigen::Vector3f, 4> pinky;
    Eigen::Matrix3f palm_frame;
    
    static constexpr int NUM_FINGERS = 5;
    
    Eigen::Vector3f keypoint(int idx) const {
        if (idx == 0) return wrist;
        if (idx >= 1 && idx <= 3) return thumb[idx - 1];
        if (idx >= 4 && idx <= 7) return index[idx - 4];
        if (idx >= 8 && idx <= 11) return middle[idx - 8];
        if (idx >= 12 && idx <= 15) return ring[idx - 12];
        if (idx >= 16 && idx <= 20) return pinky[idx - 16];
        return Eigen::Vector3f::Zero();
    }
    
    static int keypointIndex(int finger, int joint) {
        if (finger == 0) return joint;
        return 1 + (finger - 1) * 4 + joint;
    }
};

struct ManusNode {
    uint32_t id;
    Eigen::Vector3f position;
    Eigen::Quaternionf rotation;
};

class ManusUniversalPoseConverter {
public:
    static UniversalHandPose convert(
        const std::vector<ManusNode>& nodes,
        bool is_right_hand = true);
    
    static Eigen::Matrix3f computePalmFrame(
        const Eigen::Vector3f& wrist,
        const Eigen::Vector3f& index_mcp,
        const Eigen::Vector3f& pinky_mcp);
    
    static UniversalHandPose convertFromErgonomics(
        const std::vector<std::pair<std::string, float>>& ergonomics);

private:
    static constexpr float MEDIAPIPE_SCALE = 1000.0f;
};

#endif
