/*
Defines a base class for tree nodes for use in Iron Lab's Predictive Velocity Controller.
Subclasses from this base class should implement specific types of calculations.

Author: Connor Brooks
*/

#ifndef PREDIKCT_TREE_NODE_H
#define PREDIKCT_TREE_NODE_H

//includes
#include <vector>
#include <boost/shared_ptr.hpp>
#include <ros/ros.h>

namespace predikct
{

// forward declares
class RobotModel;
class MotionState;
class RewardCalculator;
class UserModel;

class TreeSpec
{
public:
    TreeSpec()
    {}
    ~TreeSpec()
    {}
    //Total number of layers in tree (root not included)
    int tree_depth;
    //Which type of user model to use. 0 for exponential straight line, 1 for random
    int user_model_type;
    //How many motion candidate are generated from each MotionCandidateNode
    int motion_candidate_branching_factor;
    //How many possible velocity commands are sampled from each UserPredictionNode
    int user_prediction_branching_factor;
    //How many possible velocity primitives to create and sample from
    int velocity_primitive_set_size;
    //How much time passes (in seconds) between each MotionCandidateNode and its children UserPredictionNodes
    double time_window;
    //How much to discount values of children MotionCandidateNodes from each UserPredictionNode (0-1)
    double temporal_discount;
};

class TreeNode
{
public:
    TreeNode(TreeNode* parent, RobotModel* robot_model, boost::shared_ptr<MotionState> state, 
        int tree_depth, TreeSpec* tree_spec, RewardCalculator* reward_calculator, UserModel* user_model)
    {
        this->parent_ = parent;
        iteration_active_ = false;
        reward_calculated_ = false;

        robot_model_ = robot_model;
        state_ = state;
        node_depth_ = tree_depth;
        tree_spec_ = tree_spec;
        reward_calc_ = reward_calculator;
        user_model_ = user_model;

        is_leaf_ = node_depth_ == tree_spec_->tree_depth;
    }
    ~TreeNode()
    {}
    //pure virtual function
    virtual std::vector<boost::shared_ptr<TreeNode>> GenerateChildren() = 0;
    virtual double CalculateReward() = 0;

    TreeNode* GetParent()
    {
        return parent_;
    }

    std::vector<boost::shared_ptr<TreeNode>> GetChildren()
    {
        return children_;
    }

    void StartChildIterator()
    {
        child_iterator_ = children_.begin();
        iteration_active_ = true;
    }

    bool GetNextChild(boost::shared_ptr<TreeNode> next_child)
    {
        if(iteration_active_ && child_iterator_ != children_.end())
        {
            next_child = *child_iterator_;
            child_iterator_++;
            return true;
        }

        return false;
    }

    double GetReward()
    {
        if(!reward_calculated_)
        {
            CalculateReward();
        }

        return reward_;
    }

protected:
    TreeNode* parent_;
    std::vector<boost::shared_ptr<TreeNode>> children_;
    bool iteration_active_;

    bool reward_calculated_;
    double reward_;

    int node_depth_;
    bool is_leaf_;

    boost::shared_ptr<MotionState> state_;
    RobotModel* robot_model_;

    TreeSpec* tree_spec_;
    RewardCalculator* reward_calc_;
    UserModel* user_model_;

    std::vector<boost::shared_ptr<TreeNode>>::iterator child_iterator_;
};

}

#endif  // PREDIKCT_TREE_NODE_H