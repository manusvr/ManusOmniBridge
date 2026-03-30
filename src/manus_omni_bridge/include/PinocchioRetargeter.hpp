#ifndef PINOCCHIO_RETARGETER_HPP
#define PINOCCHIO_RETARGETER_HPP

#include <memory>
#include <string>
#include <vector>
#include <array>
#include <Eigen/Dense>
#include <pinocchio/fwd.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>
#include <nlopt.hpp>

#include "ManusUniversalPose.hpp"

class LPFilter {
public:
    explicit LPFilter(double alpha);
    Eigen::VectorXd next(const Eigen::VectorXd& x);
    void reset();
    
private:
    double alpha_;
    Eigen::VectorXd y_;
    bool is_init_;
};

class PinocchioRetargeter {
public:
    PinocchioRetargeter(const std::string& urdf_path, bool is_o12 = false);
    
    Eigen::VectorXd retarget(const UniversalHandPose& pose);
    Eigen::VectorXd retarget(const std::vector<ManusNode>& nodes);
    Eigen::VectorXd retarget(const Eigen::Ref<const Eigen::MatrixXf>& keypoints);
    
    double computeLoss(const Eigen::VectorXd& qpos, Eigen::VectorXd& grad);
    
    void setConfig(const std::string& yaml_path);
    
    int numJoints() const { return static_cast<int>(num_joints_); }

    Eigen::MatrixXf getKeypoints() const { return current_keypoints_; } 
    
    void reset();

private:
    void initOptimizer();
    void initLinkIndices();
    
    void computeTargetVectors(const Eigen::Ref<const Eigen::MatrixXf>& keypoints);
    
    void forwardKinematics(const Eigen::VectorXd& qpos);
    
    std::array<double, 5> computePinchAlphas(const Eigen::Ref<const Eigen::MatrixXf>& keypoints);
    
    pinocchio::Model model_;
    pinocchio::Data data_;
    
    std::unique_ptr<nlopt::opt> optimizer_;
    
    int num_joints_;
    std::vector<int> tip_indices_;
    std::vector<int> pip_indices_;
    std::vector<int> dip_indices_;
    std::vector<int> wrist_idx_;
    
    std::vector<int> tip_joint_ids_;
    int wrist_joint_id_;
    
    Eigen::VectorXd last_qpos_;
    LPFilter lp_filter_;
    
    double huber_delta_ = 2.0;
    double huber_delta_dir_ = 0.5;
    double w_pos_ = 1.0;
    double w_dir_ = 10.0;
    double w_full_hand_ = 1.0;
    double norm_delta_ = 0.04;
    double scaling_ = 1.0;
    
    Eigen::Matrix<double, 5, 3> segment_scaling_;
    Eigen::Matrix<double, 5, 2> thresholds_;
    
    Eigen::Matrix<double, 5, 3> target_tip_vectors_;
    Eigen::Matrix<double, 5, 3> target_tip_dirs_;
    Eigen::Matrix<double, 15, 3> target_full_hand_vectors_;
    std::array<double, 5> pinch_alphas_;
    
    Eigen::MatrixXd target_local_dirs_;
    int index_mcp_idx_;
    int pinky_mcp_idx_;
    double target_pinch_distance_;

    Eigen::MatrixXf current_keypoints_;

    Eigen::VectorXd target_tip_distances_;

    static constexpr double M_TO_CM = 100.0;
    static constexpr int NUM_FINGERS = 5;
};

double huberLoss(double x, double delta);
double huberLossGrad(double x, double delta);

#endif
