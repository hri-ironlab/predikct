/*
Defines a model class that handles a URDF of a robot used in planning.
Robot joint positions can be passed in for forward and inverse kinematics.
Built on KDL library.

Author: Connor Brooks
*/
#include "predikct/robot_model.h"
#include <ros/ros.h>
#include <string>
#include <kdl/tree.hpp>
#include <kdl_parser/kdl_parser.hpp>

namespace predikct
{

RobotModel::RobotModel()
{
    busy_ = true;
    initialized_ = false;
    Init();
}

void RobotModel::Init()
{
    //Get root and tip joints for planning
    ros::param::get("/KCT_Controller/planning_root_link", root_link_);
    ros::param::get("/KCT_Controller/planning_tip_link", tip_link_);

    //Load URDF from parameter server
    std::string urdf_string;
    ros::param::get("/planning_robot_urdf", urdf_string);
    model_.initString(urdf_string);

    //Load KDL tree
    KDL::Tree kdl_tree;
    kdl_parser::treeFromUrdfModel(model_, kdl_tree);
    //Populate the chain
    kdl_tree.getChain(root_link_, tip_link_, kdl_chain_);

    jac_solver_ = std::make_shared<KDL::ChainJntToJacSolver>(kdl_chain_);
    jnt_to_pos_solver_ = std::make_shared<KDL::ChainFkSolverPos_recursive>(kdl_chain_);
    jnt_pos_.resize(kdl_chain_.getNrOfJoints());
    jacobian_.resize(kdl_chain_.getNrOfJoints());


    double new_joint_limit;
    for(int i = 0; i < kdl_chain_.getNrOfJoints(); ++i)
    {
        ros::param::get("/KCT_Controller/joint" + std::to_string(i) + "_pos_up_limit", new_joint_limit);
        jnt_pos_up_limits_.push_back(new_joint_limit);
        
        ros::param::get("/KCT_Controller/joint" + std::to_string(i) + "_pos_down_limit", new_joint_limit);
        jnt_pos_down_limits_.push_back(new_joint_limit);

        ros::param::get("/KCT_Controller/joint" + std::to_string(i) + "_vel_limit", new_joint_limit);
        jnt_vel_limits_.push_back(new_joint_limit);
        ROS_ERROR("joint vel limit %d: %.2f", i, jnt_vel_limits_[i]);
    }

    initialized_ = true;
    busy_ = false;
}

int RobotModel::GetNumberOfJoints()
{
    if(!initialized_){
        return -1;
    }

    return kdl_chain_.getNrOfJoints();
}

void RobotModel::GetJacobian(std::vector<double> joint_positions, Eigen::Matrix<double,6,Eigen::Dynamic>* jac)
{
    SetRobotPosition(joint_positions);
    jac->resize(jacobian_.rows(), jacobian_.columns());
    for(int i = 0; i < jacobian_.rows(); ++i)
    {
        for(int j = 0; j < jacobian_.columns(); ++j)
        {
            (*jac)(i, j) = jacobian_(i, j);
        }
    }
}

void RobotModel::GetPosition(std::vector<double> joint_positions, KDL::Frame* position)
{
    SetRobotPosition(joint_positions);
    jnt_to_pos_solver_->JntToCart(jnt_pos_, *position);

}

double RobotModel::GetJointPosUpLimit(int joint_index)
{
    return jnt_pos_up_limits_[joint_index];
}

double RobotModel::GetJointPosDownLimit(int joint_index)
{
    return jnt_pos_down_limits_[joint_index];
}

double RobotModel::GetJointVelLimit(int joint_index)
{
    return jnt_vel_limits_[joint_index];
}

//--------------------------------------------------------------------------------------------------
// Private functions

bool RobotModel::SetRobotPosition(std::vector<double> joint_positions)
{
    //Check that size of vector matches number of joints
    if(joint_positions.size() != GetNumberOfJoints())
    {
        return false;
    }

    //Update joint positions
    for(size_t i = 0; i < joint_positions.size(); i++)
    {
        jnt_pos_(i) = joint_positions[i];
    }

    //Update Jacobian
    jac_solver_->JntToJac(jnt_pos_, jacobian_);
    return true;
}

}