// includes
#include "predikct/motion_candidate_node.h"
#include "predikct/motion_state.h"
#include "predikct/user_prediction_node.h"
#include "predikct/reward_calculator.h"
#include <random>
#include <kdl/frames.hpp>

namespace predikct
{
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
    GetPseudoInverse(pseudo_inverse_);
    Eigen::VectorXd primary_movement = pseudo_inverse_ * desired_velocity_;
    bool verbose = false;
    if(verbose)
    {
        ROS_ERROR("Primary Vels: %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f", primary_movement(0), primary_movement(1), primary_movement(2), primary_movement(3), primary_movement(4), primary_movement(5), primary_movement(6));
    }

    //Generate possible movement commands for using redundant DOF within null space of pseudo-inverse
    GetNullSpace(pseudo_inverse_, null_space_);
    //Generate null space movement around following the same null movement goals from previous timesteps with added noise
    std::random_device rd;
    std::default_random_engine generator(rd());
    std::vector<std::normal_distribution<double>> distributions;
    for(int i = 0; i < state_->joint_velocities.size(); ++i)
    {
        distributions.push_back(std::normal_distribution<double>(0.0, std::max(0.3, sqrt(abs(state_->null_goals[i]))) ));
    }

    //Generate number of children according to branching factor
    Eigen::VectorXd total_movement;
    for(int i = 0; i < tree_spec_->motion_candidate_branching_factor; i++)
    {
        Eigen::VectorXd null_movement_eigen(state_->joint_positions.size());
        for(int j = 0; j < state_->joint_positions.size(); j++)
        {
            if(i == 0){
                null_movement_eigen(j) = state_->null_goals[j];
            } else {
                null_movement_eigen(j) = state_->null_goals[j] + distributions[j](generator);
            }
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
    bool verbose = false;
    state_->position.M.GetQuaternion(x, y, z, w);
    if(verbose)
    {
        ROS_ERROR("Getting reward from motion candidate node at depth %d", node_depth_);
        ROS_ERROR("Starting from pose: %.2f %.2f %.2f / %.2f %.2f %.2f %.2f", state_->position.p.x(), state_->position.p.y(), state_->position.p.z(), x, y, z, w);
        ROS_ERROR("Starting from joint vels: %.2f %.2f %.2f %.2f %.2f %.2f %.2f", state_->joint_velocities[0], state_->joint_velocities[1], state_->joint_velocities[2], state_->joint_velocities[3], state_->joint_velocities[4], state_->joint_velocities[5], state_->joint_velocities[6]);
        ROS_ERROR("Desired Velocity: %.2f %.2f %.2f %.2f %.2f %.2f", desired_velocity_raw_[0], desired_velocity_raw_[1], desired_velocity_raw_[2], desired_velocity_raw_[3], desired_velocity_raw_[4], desired_velocity_raw_[5]);
    }
    ideal_position.M.GetQuaternion(x, y, z, w);
    if(verbose)
    {
        ROS_ERROR("Ideal pose: %.2f %.2f %.2f / %.2f %.2f %.2f %.2f", ideal_position.p.x(), ideal_position.p.y(), ideal_position.p.z(), x, y, z, w);
    }
    double max_reward = reward_calc_->EvaluateMotionCandidate(robot_model_, state_, motion_candidates_[0], &ideal_position, verbose);
    if(!is_leaf_){
        max_reward += children_[0]->GetReward();
        if(verbose){
            ROS_ERROR("Child reward: %.2f\nTotal reward:  %.2f", children_[0] ->GetReward(), max_reward);
        }
    }
    best_candidate_index_ = 0;
    for(int i = 1; i < motion_candidates_.size(); i++)
    {
        double new_reward = reward_calc_->EvaluateMotionCandidate(robot_model_, state_, motion_candidates_[i], &ideal_position, verbose);
        if(!is_leaf_)
        {
            new_reward += children_[i]->GetReward();
            if(verbose)
            {
                ROS_ERROR("Child reward: %.2f\nTotal reward:  %.2f", children_[i] ->GetReward(), new_reward);
            }
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
    GetPseudoInverse(pseudo_inverse_);
    Eigen::VectorXd primary_movement = pseudo_inverse_ * desired_velocity_;

    for(int i = 0; i < primary_movement.size(); ++i)
    {
        joint_velocities->push_back(primary_movement(i));
    }
}

//--------------------------------------------------------------------------------------------------
// Linear algebra functions

void MotionCandidateNode::GetPseudoInverse(Eigen::Matrix<double,Eigen::Dynamic,6>& pseudo_inverse)
{
    Eigen::Matrix<double,Eigen::Dynamic,6> jacobian_transpose = state_->jacobian.transpose();
    pseudo_inverse = jacobian_transpose * (state_->jacobian * jacobian_transpose).inverse();
}

void MotionCandidateNode::GetNullSpace(Eigen::Matrix<double,Eigen::Dynamic,6>& pseudo_inverse, Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic>& null_space)
{
    Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic> ident = Eigen::Matrix<double,Eigen::Dynamic, Eigen::Dynamic>::Identity(pseudo_inverse.rows(),state_->jacobian.cols());
    null_space = ident - pseudo_inverse * state_->jacobian;
}

}