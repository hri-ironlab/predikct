// includes
#include "predikct/motion_candidate_node.h"
#include "predikct/motion_state.h"
#include "predikct/user_prediction_node.h"
#include "predikct/reward_calculator.h"
#include <random>
#include <kdl/frames.hpp>
#include <Eigen/Eigenvalues>

namespace predikct
{

struct normal_random_variable
{
    normal_random_variable(Eigen::MatrixXd const& covar)
        : normal_random_variable(Eigen::VectorXd::Zero(covar.rows()), covar)
    {}

    normal_random_variable(Eigen::VectorXd const& mean, Eigen::MatrixXd const& covar)
        : mean(mean)
    {
        Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eigenSolver(covar);
        transform = eigenSolver.eigenvectors() * eigenSolver.eigenvalues().cwiseSqrt().asDiagonal();
    }

    Eigen::VectorXd mean;
    Eigen::MatrixXd transform;

    Eigen::VectorXd operator()() const
    {
        static std::mt19937 gen{ std::random_device{}() };
        static std::normal_distribution<> dist;

        return mean + transform * Eigen::VectorXd{ mean.size() }.unaryExpr([&](double x) { return dist(gen); });
    }
};

MotionCandidateNode::MotionCandidateNode(TreeNode* parent, RobotModel* robot_model, boost::shared_ptr<MotionState> state, int tree_depth, 
    TreeSpec* tree_spec, RewardCalculator* reward_calculator, UserModel* user_model, std::vector<double>* desired_velocity) 
    : TreeNode(parent, robot_model, state, tree_depth, tree_spec, reward_calculator, user_model)
{
    desired_velocity_ = Eigen::Map<Eigen::VectorXd>(&((*desired_velocity)[0]), desired_velocity->size());
    state->CalculateJacobian(robot_model);
    candidates_generated_ = false;
    desired_velocity_raw_ = std::vector<double>(desired_velocity->size());
    desired_movement_ = std::vector<double>(desired_velocity->size());
    for(int i = 0; i < desired_velocity_raw_.size(); i++)
    {
        desired_velocity_raw_[i] = (*desired_velocity)[i];
        desired_movement_[i] = ((*desired_velocity)[i])*tree_spec_->time_window;
    }
    is_leaf_ = tree_depth >= tree_spec->tree_depth;
}

// Generate a set of set of joint motion candidates,
//  each of which leads to a User Prediction node
std::vector<boost::shared_ptr<TreeNode>> MotionCandidateNode::GenerateChildren()
{
    children_.clear();
    motion_candidates_.clear();

    //Calculate movement that satisfies desired velocity using pseudo-inverse
    Eigen::VectorXd primary_movement = state_->pseudo_inverse * desired_velocity_;

    //Generate possible movement commands for using redundant DOF within null space of pseudo-inverse
    GetNullSpace();
    //Generate null space movement around following the same null movement goals from previous timesteps with added noise sampled from a multivariate distribution with mean of previous null space motion and covariance = identity matrix
    Eigen::VectorXd base_null_eigen(state_->joint_positions.size());
    Eigen::VectorXd target_null_eigen(state_->joint_positions.size());
    Eigen::VectorXd mean_null_eigen(state_->joint_positions.size());
    bool all_zeros = true;
    for(int j = 0; j < state_->joint_positions.size(); j++)
    {
        base_null_eigen(j) = state_->null_goals[j];
        if(base_null_eigen(j) != 0.0)
        {
            all_zeros = false;
        }
    }
    // Find the mean of the last null vector and the current gradient of the manipulability index
    Eigen::Matrix<double,Eigen::Dynamic, Eigen::Dynamic> covariance = Eigen::Matrix<double,Eigen::Dynamic, Eigen::Dynamic>::Identity(state_->joint_positions.size(),state_->joint_positions.size());

    // CALCULATION OF GRADIENT OF MANIPULABILITY INDEX
    // Calculation of derivative source: https://arxiv.org/pdf/1908.02963.pdf (page 3, left column)
    // Step 1: Calculate the manipulability index
    state_->CalculateManipulability(robot_model_);
    Eigen::Matrix<double,6,Eigen::Dynamic> jac_dot;
    double k_gradient_weight = 25;
    for(int j = 0; j < state_->joint_positions.size(); j++)
    {
        // Step 2: Find the partial derivative of the Jacobian with respect to the current joint
        robot_model_->GetJacobianDot(state_->joint_positions, j, &jac_dot);
        // Steps 3 & 4: Find the trace of partial derivative of Jacobian times pseudoinverse, then multiply answer from Step 3 by Step 1 to get gradient
        target_null_eigen(j) = k_gradient_weight * state_->manipulability * ((jac_dot * state_->pseudo_inverse).trace());

        if(all_zeros)
        {
            mean_null_eigen(j) = target_null_eigen(j);
            covariance(j,j) = 0.1;
        }
        else
        {
            mean_null_eigen(j) = (base_null_eigen(j) + target_null_eigen(j)) / 2.0;
            covariance(j,j) = pow(std::abs(base_null_eigen(j) - target_null_eigen(j)), 2) / 2.0;
        }
    }

    normal_random_variable noise_sample { mean_null_eigen, covariance };

    //Generate number of children according to branching factor
    Eigen::VectorXd total_movement;
    for(int i = 0; i < tree_spec_->motion_candidate_branching_factor; i++)
    {
        Eigen::VectorXd null_movement_eigen(state_->joint_positions.size());
        if(i == 0){
            null_movement_eigen = base_null_eigen;
        } else {
            null_movement_eigen = noise_sample();
        }
        
        total_movement = primary_movement + null_space_*null_movement_eigen;

        //Create new motion candidate
        std::vector<double> motion_candidate;
        motion_candidate.resize(total_movement.size());
        Eigen::VectorXd::Map(&motion_candidate[0], total_movement.size()) = total_movement;
        std::vector<double> null_goals;
        null_goals.resize(null_movement_eigen.size());
        Eigen::VectorXd::Map(&null_goals[0], null_movement_eigen.size()) = null_movement_eigen;
        boost::shared_ptr<MotionState> new_motion_candidate(new MotionState(&(state_->joint_positions), &(state_->joint_velocities), &motion_candidate, &null_goals, tree_spec_->time_window, robot_model_, node_depth_ * tree_spec_->time_window));
        motion_candidates_.push_back(new_motion_candidate);
        if(!is_leaf_)
        {
            //Create new child node
            children_.push_back(boost::shared_ptr<TreeNode> (new UserPredictionNode(this, robot_model_, new_motion_candidate, 
                node_depth_, tree_spec_, reward_calc_, user_model_, &desired_velocity_raw_)));
        }
    }

    candidates_generated_ = true;
    return children_;
}

double MotionCandidateNode::CalculateReward()
{
    if(!candidates_generated_)
    {
        GenerateChildren();
    }

    //Calculate ideal position after applying desired velocity
    state_->CalculatePosition(robot_model_);
    KDL::Frame ideal_position( KDL::Rotation::RPY(desired_movement_[3], desired_movement_[4], desired_movement_[5]) * state_->position.M, 
        state_->position.p + KDL::Vector(desired_movement_[0], desired_movement_[1], desired_movement_[2]));

    double x, y, z, w;
    state_->position.M.GetQuaternion(x, y, z, w);
    ideal_position.M.GetQuaternion(x, y, z, w);
    double max_reward = reward_calc_->EvaluateMotionCandidate(robot_model_, state_, motion_candidates_[0], &ideal_position);
    if(!is_leaf_){
        max_reward += children_[0]->GetReward();
    }
    best_candidate_index_ = 0;
    for(int i = 1; i < motion_candidates_.size(); i++)
    {
        double new_reward = reward_calc_->EvaluateMotionCandidate(robot_model_, state_, motion_candidates_[i], &ideal_position);
        if(!is_leaf_)
        {
            new_reward += children_[i]->GetReward();
        }
        if(new_reward > max_reward)
        {
            max_reward = new_reward;
            best_candidate_index_ = i;
        }
    }
    reward_ = max_reward;

    reward_calculated_ = true;
    return reward_;
}

void MotionCandidateNode::ChooseMotionCandidate(std::vector<double>* joint_velocities, std::vector<double>* null_vector)
{
    joint_velocities->clear();
    null_vector->clear();
    if(!reward_calculated_)
    {
        CalculateReward();
    }

    for(int i = 0; i < (*motion_candidates_[best_candidate_index_]).commanded_velocities.size(); ++i)
    {
        joint_velocities->push_back((*motion_candidates_[best_candidate_index_]).commanded_velocities[i]);
        null_vector->push_back((*motion_candidates_[best_candidate_index_]).null_goals[i]);
    }
}

void MotionCandidateNode::GetBaseline(std::vector<double>* joint_velocities)
{
    joint_velocities->clear();
    state_->CalculateJacobian(robot_model_);
    Eigen::VectorXd primary_movement = state_->pseudo_inverse * desired_velocity_;

    for(int i = 0; i < primary_movement.size(); ++i)
    {
        joint_velocities->push_back(primary_movement(i));
    }
}

//--------------------------------------------------------------------------------------------------
// Linear algebra functions
void MotionCandidateNode::GetNullSpace()
{
    Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic> ident = Eigen::Matrix<double,Eigen::Dynamic, Eigen::Dynamic>::Identity(state_->pseudo_inverse.rows(),state_->jacobian.cols());
    null_space_ = ident - state_->pseudo_inverse * state_->jacobian;
}

}
