/*
Defines a tree node subclass that has a state and velocity primitive given from parent
 and has a set of motion primitives as children

Author: Connor Brooks
*/

#ifndef PREDIKCT_MOTION_CANDIDATE_NODE_H
#define PREDIKCT_MOTION_CANDIDATE_NODE_H

// includes
#include "predikct/tree_node.h"

#include <Eigen/Core>
#include <boost/shared_ptr.hpp>

namespace predikct
{

class MotionCandidateNode : public TreeNode
{
public:
    MotionCandidateNode(TreeNode* parent, RobotModel* robot_model, boost::shared_ptr<MotionState> state, int tree_depth, TreeSpec* tree_spec, 
        RewardCalculator* reward_calculator, UserModel* user_model, std::vector<double>* desired_velocity);
    ~MotionCandidateNode()
    {}
    std::vector<boost::shared_ptr<TreeNode>> GenerateChildren();

    double CalculateReward();

    void ChooseMotionCandidate(std::vector<double>* joint_velocities, std::vector<double>* null_vector);

    void GetBaseline(std::vector<double>* joint_velocities);

private:
    void GetNullSpace();

    std::vector<boost::shared_ptr<MotionState>> motion_candidates_;

    Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic> null_space_;
    Eigen::VectorXd desired_velocity_;
    std::vector<double> desired_velocity_raw_;
    std::vector<double> desired_movement_;

    int best_candidate_index_;

    bool candidates_generated_;
};

}

#endif  // PREDIKCT_MOTION_CANDIDATE_NODE_H
