#include "PinocchioRetargeter.hpp"
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <iostream>
#include <fstream>
#include <cstdio>

static bool g_debug = false;
#define DEBUG_PRINT(fmt, ...) do { if (g_debug) { fprintf(stderr, "[DEBUG] " fmt "\n", ##__VA_ARGS__); } } while(0)

LPFilter::LPFilter(double alpha) : alpha_(alpha), is_init_(false) {}

Eigen::VectorXd LPFilter::next(const Eigen::VectorXd& x) {
    if (!is_init_ || y_.size() != x.size()) {
        y_.resize(x.size());
        y_ = x;
        is_init_ = true;
        return y_;
    }
    y_ = y_ + alpha_ * (x - y_);
    return y_;
}

void LPFilter::reset() {
    is_init_ = false;
}

double huberLoss(double x, double delta) {
    double abs_x = std::abs(x);
    if (abs_x <= delta) {
        return 0.5 * x * x;
    }
    return delta * (abs_x - 0.5 * delta);
}

double huberLossGrad(double x, double delta) {
    double abs_x = std::abs(x);
    if (abs_x <= delta) {
        return x;
    }
    return delta * (x > 0 ? 1.0 : -1.0);
}

PinocchioRetargeter::PinocchioRetargeter(const std::string& urdf_path, bool is_o12)
    : lp_filter_(0.4)
{
    DEBUG_PRINT("Constructor: building model from %s", urdf_path.c_str());
    pinocchio::urdf::buildModel(urdf_path, model_);
    DEBUG_PRINT("Constructor: model built with nq=%d, nframes=%d", (int)model_.nq, (int)model_.frames.size());
    data_ = pinocchio::Data(model_);
    DEBUG_PRINT("================ PINOCCHIO JOINT MAPPING ================");
    for (size_t i = 1; i < model_.joints.size(); ++i) { // Start at 1 to skip the 'universe' root joint
        DEBUG_PRINT("Joint %zu: %s | idx_q: %d", 
            i, 
            model_.names[i].c_str(), 
            model_.joints[i].idx_q());
    }
    DEBUG_PRINT("=========================================================");
    
    num_joints_ = static_cast<int>(model_.nq);
    
    DEBUG_PRINT("Constructor: model_.nq=%d, model_.nv=%d", (int)model_.nq, (int)model_.nv);
    
    segment_scaling_.setOnes();
    thresholds_.setOnes();
    thresholds_.col(0) *= 2.0;
    thresholds_.col(1) *= 4.0;
    
    initLinkIndices();
    initOptimizer();
}

void PinocchioRetargeter::initLinkIndices() {
    tip_indices_.resize(5);
    pip_indices_.resize(5);
    dip_indices_.resize(5);
    wrist_idx_.resize(1);
    
    tip_joint_ids_.resize(5);
    wrist_joint_id_ = 0;
    
    const std::vector<std::string> tip_names = {
        "R_thumb_tip", "R_index_tip", "R_middle_tip", "R_ring_tip", "R_pinky_tip"
    };
    const std::vector<std::string> pip_names = {
        "R_thumb_pip_link", "R_index_pip_link", "R_middle_pip_link", "R_ring_pip_link", "R_pinky_pip_link"
    };
    const std::vector<std::string> dip_names = {
        "R_thumb_dip_link", "R_index_dip_link", "R_middle_dip_link", "R_ring_dip_link", "R_pinky_dip_link"
    };
    
    for (int i = 0; i < 5; ++i) {
        tip_indices_[i] = model_.getFrameId(tip_names[i]);
        pip_indices_[i] = model_.getFrameId(pip_names[i]);
        dip_indices_[i] = model_.getFrameId(dip_names[i]);
    }
    
    wrist_idx_[0] = model_.getFrameId("R_palm");
    
    tip_joint_ids_[0] = model_.getJointId("R_thumb_tip_joint");
    tip_joint_ids_[1] = model_.getJointId("R_index_tip_joint");
    tip_joint_ids_[2] = model_.getJointId("R_middle_tip_joint");
    tip_joint_ids_[3] = model_.getJointId("R_ring_tip_joint");
    tip_joint_ids_[4] = model_.getJointId("R_pinky_tip_joint");


    tip_joint_ids_[0] = model_.getFrameId("R_thumb_tip_joint");
    tip_joint_ids_[1] = model_.getFrameId("R_index_tip_joint");
    tip_joint_ids_[2] = model_.getFrameId("R_middle_tip_joint");
    tip_joint_ids_[3] = model_.getFrameId("R_ring_tip_joint");
    tip_joint_ids_[4] = model_.getFrameId("R_pinky_tip_joint");

    index_mcp_idx_ = model_.getFrameId("R_index_abad_link"); 
    pinky_mcp_idx_ = model_.getFrameId("R_pinky_abad_link");

    wrist_joint_id_ = model_.getJointId("R_palm_joint");
    
    DEBUG_PRINT("initLinkIndices: tip joint ids = %d %d %d %d %d", tip_joint_ids_[0], tip_joint_ids_[1], tip_joint_ids_[2], tip_joint_ids_[3], tip_joint_ids_[4]);
    DEBUG_PRINT("initLinkIndices: wrist joint id = %d", wrist_joint_id_);
}

void PinocchioRetargeter::initOptimizer() {
    optimizer_ = std::make_unique<nlopt::opt>(nlopt::LD_SLSQP, num_joints_);
    
    optimizer_->set_maxeval(15);
    optimizer_->set_ftol_abs(1e-2);
    optimizer_->set_xtol_rel(1e-2);
    
    auto lower = model_.lowerPositionLimit;
    auto upper = model_.upperPositionLimit;
    
    std::vector<double> lower_vec(lower.data(), lower.data() + lower.size());
    std::vector<double> upper_vec(upper.data(), upper.data() + upper.size());
    
    optimizer_->set_lower_bounds(lower_vec);
    optimizer_->set_upper_bounds(upper_vec);
}



std::array<double, 5> PinocchioRetargeter::computePinchAlphas(
    const Eigen::Ref<const Eigen::MatrixXf>& keypoints) 
{
    Eigen::Vector3f thumb_tip = keypoints.col(4);
    std::array<double, 5> alphas;
    
    for (int i = 0; i < 4; ++i) {
        Eigen::Vector3f finger_tip = keypoints.col(8 + i * 4);
        double dist = (finger_tip - thumb_tip).norm() * M_TO_CM;
        
        double d1 = thresholds_(i, 0);
        double d2 = thresholds_(i, 1);
        
        alphas[i] = std::max(0.0, std::min(0.7, (d2 - dist) / (d2 - d1 + 1e-8)));
    }
    
    alphas[4] = std::max({alphas[0], alphas[1], alphas[2], alphas[3]});
    
    return alphas;
}

void PinocchioRetargeter::forwardKinematics(const Eigen::VectorXd& qpos) {
    pinocchio::forwardKinematics(model_, data_, qpos);
    pinocchio::updateGlobalPlacements(model_, data_);
    for (size_t fid = 0; fid < model_.frames.size(); ++fid) {
        const pinocchio::Frame& frame = model_.frames[fid];
        if (frame.parentJoint < model_.joints.size()) {
            data_.oMf[fid] = data_.oMi[frame.parentJoint] * frame.placement;
        }
    }
}

void PinocchioRetargeter::computeTargetVectors(const Eigen::Ref<const Eigen::MatrixXf>& keypoints) {
    Eigen::Vector3d wrist = keypoints.col(0).cast<double>();
    int tip_cols[5] = {4, 8, 12, 16, 20};

    target_local_dirs_.resize(5, 3);
    target_tip_distances_.resize(5);

    // Calculate Robot Max Reach
    static std::vector<double> robot_max_reach(5, -1.0);
    if (robot_max_reach[0] < 0) {
        Eigen::VectorXd q_flat = Eigen::VectorXd::Zero(model_.nq);
        pinocchio::Data data_flat(model_); 
        
        pinocchio::forwardKinematics(model_, data_flat, q_flat);
        pinocchio::updateGlobalPlacements(model_, data_flat);
        for (size_t fid = 0; fid < model_.frames.size(); ++fid) {
            const pinocchio::Frame& frame = model_.frames[fid];
            if (frame.parentJoint < model_.joints.size()) {
                data_flat.oMf[fid] = data_flat.oMi[frame.parentJoint] * frame.placement;
            }
        }
        
        Eigen::Vector3d r_w = data_flat.oMf[wrist_idx_[0]].translation();
        
        for (int i = 0; i < 5; ++i) {
            robot_max_reach[i] = (data_flat.oMf[tip_indices_[i]].translation() - r_w).norm();
        }
        
        DEBUG_PRINT("Robot Max Reach: Thumb=%.3f, Index=%.3f, Mid=%.3f, Ring=%.3f, Pinky=%.3f", 
            robot_max_reach[0], robot_max_reach[1], robot_max_reach[2], robot_max_reach[3], robot_max_reach[4]);
    }

    for (int i = 0; i < 5; ++i) {
        Eigen::Vector3d tip = keypoints.col(tip_cols[i]).cast<double>();
        
        // Target Direction (Cosine Similarity)
        target_local_dirs_.row(i) = (tip - wrist).normalized().transpose();

        // Calculate Human Max Reach (Sum of bone segment lengths)
        double human_max_reach = 0.0;
        int start_idx = 1 + i * 4; // Thumb=1, Index=5, Middle=9, etc.
        human_max_reach += (keypoints.col(start_idx).cast<double>() - wrist).norm(); // Wrist to MCP
        human_max_reach += (keypoints.col(start_idx+1) - keypoints.col(start_idx)).norm(); // MCP to PIP
        human_max_reach += (keypoints.col(start_idx+2) - keypoints.col(start_idx+1)).norm(); // PIP to DIP
        human_max_reach += (keypoints.col(start_idx+3) - keypoints.col(start_idx+2)).norm(); // DIP to Tip

        // Current Human Distance (Wrist straight to Tip)
        double human_current_dist = (tip - wrist).norm();

        // Calculate scaled target distance for the robot
        target_tip_distances_(i) = human_current_dist * (robot_max_reach[i] / (human_max_reach + 1e-6));    }
}

double PinocchioRetargeter::computeLoss(const Eigen::VectorXd& qpos, Eigen::VectorXd& grad) {
    auto evaluate_pose_loss = [&](const Eigen::VectorXd& q) -> double {
        forwardKinematics(q); 
        double loss = 0.0;

        Eigen::Vector3d r_wrist = data_.oMf[wrist_idx_[0]].translation();
        Eigen::Vector3d h_wrist = current_keypoints_.col(0).cast<double>();

        // --- Unified 3D Target Position Loss ---
        for (int i = 0; i < 5; ++i) {
            Eigen::Vector3d r_tip = data_.oMf[tip_indices_[i]].translation();
            Eigen::Vector3d target_dir = target_local_dirs_.row(i).transpose();
            double target_dist = target_tip_distances_(i);
            Eigen::Vector3d target_pos = r_wrist + (target_dir * target_dist);
            
            // loss for the Euclidean distance between current robot tip and target tip.
            // This pulls the finger open smoothly
            loss += 2000.0 * (r_tip - target_pos).squaredNorm();
        }

        // --- PINCH OPTIMIZATION ---
        Eigen::Vector3d r_thumb_tip = data_.oMf[tip_indices_[0]].translation();
        for (int i = 1; i < 5; ++i) {

            double pinch_weight = pinch_alphas_[i - 1]; 
            if (pinch_weight > 0.01) {
                Eigen::Vector3d r_finger_tip = data_.oMf[tip_indices_[i]].translation();
                
                // loss for the distance between the Thumb tip and THIS Finger's tip.
                // The closer the human fingers get, the stronger this magnetic pull becomes.
                loss += 5000.0 * pinch_weight * (r_finger_tip - r_thumb_tip).squaredNorm();
            }
        } 

        // --- THUMB OPPOSITION (ROLL) ---
        // find strongest pinch happening across all 4 fingers.
        double max_pinch = std::max({pinch_alphas_[0], pinch_alphas_[1], pinch_alphas_[2], pinch_alphas_[3]});
        if (max_pinch > 0.01) {
            double desired_roll = max_pinch * 1.0; 
            // apply loss based thumb orientation
            //loss += 500.0 * std::pow(q(11) - desired_roll, 2); //disabled for now.
        }

        // --- SOFT CONSTRAINTS FOR MIMIC JOINTS ---
        double mimic_loss = 0.0;
        // Thumb (MCP = 13, PIP = 14, DIP = 15)
        mimic_loss += std::pow(q(14) - q(13) * 1.33, 2);   // Thumb PIP mimics MCP
        mimic_loss += std::pow(q(15) - q(13) * 1.30, 2);   // Thumb DIP mimics MCP
        // Index (PIP = 1, DIP = 2)
        mimic_loss += std::pow(q(2) - q(1) * 1.097, 2);    // Index DIP mimics PIP
        // Middle (PIP = 3, DIP = 4)
        mimic_loss += std::pow(q(4) - q(3) * 1.097, 2);    // Middle DIP mimics PIP
        // Ring (PIP = 9, DIP = 10)
        mimic_loss += std::pow(q(10) - q(9) * 1.097, 2);   // Ring DIP mimics PIP
        // Pinky (PIP = 6, DIP = 7)
        mimic_loss += std::pow(q(7) - q(6) * 1.097, 2);    // Pinky DIP mimics PIP
        loss += 200.0 * mimic_loss;

        return loss;
    };

    // Calculate the center loss
    double base_loss = evaluate_pose_loss(qpos);
    double actual_loss = base_loss;
    
    if (last_qpos_.size() == qpos.size()) {
        actual_loss += norm_delta_ * (qpos - last_qpos_).squaredNorm();
    }

    if (grad.size() > 0) {
        double eps = 1e-4; 
        for (int j = 0; j < num_joints_; ++j) {
            Eigen::VectorXd qpos_plus = qpos;
            qpos_plus(j) += eps;
            double l_plus = evaluate_pose_loss(qpos_plus);
            
            grad(j) = (l_plus - base_loss) / eps;
            if (last_qpos_.size() == qpos.size()) {
                grad(j) += 2.0 * norm_delta_ * (qpos(j) - last_qpos_(j));
            }
        }
        forwardKinematics(qpos); 
    }
    return actual_loss;
}

Eigen::VectorXd PinocchioRetargeter::retarget(const UniversalHandPose& pose) {
    Eigen::MatrixXf keypoints(3, 21);
    
    keypoints.col(0) = pose.wrist;
    for (int i = 0; i < 4; ++i) keypoints.col(1 + i) = pose.thumb[i];
    for (int i = 0; i < 4; ++i) keypoints.col(5 + i) = pose.index[i];
    for (int i = 0; i < 4; ++i) keypoints.col(9 + i) = pose.middle[i];
    for (int i = 0; i < 4; ++i) keypoints.col(13 + i) = pose.ring[i];
    for (int i = 0; i < 4; ++i) keypoints.col(17 + i) = pose.pinky[i];
    
    DEBUG_PRINT("retarget(pose): wrist = %f %f %f", pose.wrist(0), pose.wrist(1), pose.wrist(2));
    DEBUG_PRINT("retarget(pose): thumb tip = %f %f %f", pose.thumb[0](0), pose.thumb[0](1), pose.thumb[0](2));
    DEBUG_PRINT("retarget(pose): index tip = %f %f %f", pose.index[0](0), pose.index[0](1), pose.index[0](2));
    
    return retarget(keypoints);
}



Eigen::VectorXd PinocchioRetargeter::retarget(const std::vector<ManusNode>& nodes) {
    Eigen::MatrixXf keypoints(3, 21);
    
    keypoints.col(0) = nodes[0].position; // Wrist
    
    // Thumb
    keypoints.col(1) = nodes[1].position;
    keypoints.col(2) = nodes[2].position;
    keypoints.col(3) = nodes[3].position;
    keypoints.col(4) = nodes[4].position;
    
    // Index (Skip metacarpal ID 5)
    keypoints.col(5) = nodes[6].position;
    keypoints.col(6) = nodes[7].position;
    keypoints.col(7) = nodes[8].position;
    keypoints.col(8) = nodes[9].position;
    
    // Middle
    keypoints.col(9) = nodes[11].position;
    keypoints.col(10) = nodes[12].position;
    keypoints.col(11) = nodes[13].position;
    keypoints.col(12) = nodes[14].position;
    
    // Ring
    keypoints.col(13) = nodes[16].position;
    keypoints.col(14) = nodes[17].position;
    keypoints.col(15) = nodes[18].position;
    keypoints.col(16) = nodes[19].position;
    
    // Pinky
    keypoints.col(17) = nodes[21].position;
    keypoints.col(18) = nodes[22].position;
    keypoints.col(19) = nodes[23].position;
    keypoints.col(20) = nodes[24].position;

    current_keypoints_ = keypoints; // Save for RViz
    return retarget(keypoints);
}

struct OptData {
    PinocchioRetargeter* retargeter;
};

double nloptObjective(unsigned int n, const double* x, double* grad, void* data) {
    OptData* opt_data = static_cast<OptData*>(data);
    Eigen::VectorXd qpos = Eigen::Map<const Eigen::VectorXd>(x, n);
    
    Eigen::VectorXd grad_eigen; 
    if (grad) {
        grad_eigen.resize(n); 
    }
    
    double loss = opt_data->retargeter->computeLoss(qpos, grad_eigen);
    if (grad) {
        for (unsigned int i = 0; i < n; ++i) {
            grad[i] = grad_eigen(i);
        }
    }
    return loss;
}

Eigen::VectorXd PinocchioRetargeter::retarget(const Eigen::Ref<const Eigen::MatrixXf>& keypoints) {
    computeTargetVectors(keypoints);
    pinch_alphas_ = computePinchAlphas(keypoints);
    
    static Eigen::MatrixXf last_keypoints;
    static bool is_first = true;
    if (is_first || (keypoints - last_keypoints).norm() > 0.001) {
        DEBUG_PRINT("retarget: keypoints CHANGED! col4 = %f %f %f", keypoints.col(4)(0), keypoints.col(4)(1), keypoints.col(4)(2));
        DEBUG_PRINT("retarget: target_tip_vectors_ row0 = %f %f %f", target_tip_vectors_(0,0), target_tip_vectors_(0,1), target_tip_vectors_(0,2));
        last_keypoints = keypoints;
        is_first = false;
    }
    
    Eigen::VectorXd init_qpos;
    if (last_qpos_.size() == 0) {
        init_qpos = (model_.lowerPositionLimit + model_.upperPositionLimit) / 2.0;
        DEBUG_PRINT("retarget: init qpos from limits, size=%d", (int)init_qpos.size());
    } else {
        init_qpos = last_qpos_;
        DEBUG_PRINT("retarget: using last_qpos_, size=%d", (int)init_qpos.size());
    }
    
    std::vector<double> x_init(init_qpos.data(), init_qpos.data() + init_qpos.size());

    auto lower_bounds = model_.lowerPositionLimit;
    auto upper_bounds = model_.upperPositionLimit;
    double margin = 1e-5;
    
    for (size_t i = 0; i < x_init.size(); ++i) {
        if (x_init[i] <= lower_bounds[i]) {
            x_init[i] = lower_bounds[i] + margin;
        } else if (x_init[i] >= upper_bounds[i]) {
            x_init[i] = upper_bounds[i] - margin;
        }
    }

    double opt_f;
    OptData opt_data;
    opt_data.retargeter = this;
    
    optimizer_->set_min_objective(nloptObjective, &opt_data);
    
    try {
        std::vector<double> x_opt = x_init;
        DEBUG_PRINT("retarget: calling optimizer optimize");
        optimizer_->optimize(x_opt, opt_f);
        DEBUG_PRINT("retarget: optimize done, opt_f=%f", opt_f);
        last_qpos_ = Eigen::Map<Eigen::VectorXd>(x_opt.data(), x_opt.size());
        DEBUG_PRINT("retarget: last_qpos_ = %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f", 
            last_qpos_(0), last_qpos_(1), last_qpos_(2), last_qpos_(3), last_qpos_(4),
            last_qpos_(5), last_qpos_(6), last_qpos_(7), last_qpos_(8), last_qpos_(9),
            last_qpos_(10), last_qpos_(11), last_qpos_(12), last_qpos_(13), last_qpos_(14), last_qpos_(15));
    } catch (const std::exception& e) {
        std::cerr << "Optimization failed: " << e.what() << std::endl;
        last_qpos_ = init_qpos;
    }
    
    return lp_filter_.next(last_qpos_);
}

void PinocchioRetargeter::reset() {
    lp_filter_.reset();
    last_qpos_ = Eigen::VectorXd();
}

void PinocchioRetargeter::setConfig(const std::string& yaml_path) {
    std::ifstream file(yaml_path);
    if (!file.is_open()) {
        std::cerr << "Could not open config file: " << yaml_path << std::endl;
        return;
    }
    
    std::cout << "Config loaded from: " << yaml_path << std::endl;
}
