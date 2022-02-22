/*
Defines a tree node subclass that takes in a state and motion primitive given from parent
 and has a new state as a child

Author: Connor Brooks
*/

#ifndef PREDIKCT_USER_PREDICTION_NODE_H
#define PREDIKCT_USER_PREDICTION_NODE_H

// includes
#include "predikct/tree_node.h"
#include <boost/shared_ptr.hpp>

namespace predikct
{

class UserPredictionNode : public TreeNode
{
public:
    UserPredictionNode(TreeNode* parent, RobotModel* robot_model, boost::shared_ptr<MotionState> state, int tree_depth, TreeSpec* tree_spec, 
        RewardCalculator* reward_calculator, UserModel* user_model, std::vector<double>* current_velocity);
    ~UserPredictionNode()
    {}

    std::vector<boost::shared_ptr<TreeNode>> GenerateChildren();

    double CalculateReward();

private:
    bool children_generated_;
    std::vector<double> current_velocity_;
    std::vector<std::vector<double>> velocity_primitives_;

    std::vector<std::vector<double>> sampled_primitives_;
    std::vector<double> sampled_probabilities_;
};

}

#endif  // PREDIKCT_USER_PREDICTION_NODE_H
