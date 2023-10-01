/*
Defines a state class that maintains robot joint positions and implements forward kinematics.
Built on KDL library.

Author: Connor Brooks
*/

#ifndef PREDIKCT_ROBOT_MODEL_H
#define PREDIKCT_ROBOT_MODEL_H

#include <string>
#include <vector>

#include <ros/ros.h>

#include <urdf/model.h>

#include <kdl/chain.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/frames.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainjnttojacdotsolver.hpp>

namespace predikct
{

class RobotModel
{
public:
    RobotModel();
    ~RobotModel()
    {}

    void Init();

    int GetNumberOfJoints();

    void GetJacobian(std::vector<double> joint_positions, Eigen::Matrix<double,6,Eigen::Dynamic>* jac);

    void GetJacobianDot(std::vector<double> joint_positions, int joint_index, Eigen::Matrix<double,6,Eigen::Dynamic>* jac_dot);

    void GetPosition(std::vector<double> joint_positions, KDL::Frame* position);

    double GetJointPosUpLimit(int joint_index);

    double GetJointPosDownLimit(int joint_index);

    double GetJointVelLimit(int joint_index);

private:
    bool SetRobotPosition(std::vector<double> joint_positions);

    bool busy_;
    bool initialized_;

    std::string root_link_;
    std::string tip_link_;

    urdf::Model model_;

    KDL::Chain kdl_chain_;
    std::shared_ptr<KDL::ChainJntToJacSolver> jac_solver_;
    std::shared_ptr<KDL::ChainJntToJacDotSolver> jac_dot_solver_;
    std::shared_ptr<KDL::ChainFkSolverPos_recursive> jnt_to_pos_solver_;
    KDL::JntArray jnt_pos_;
    KDL::Jacobian jacobian_;
    KDL::Jacobian jacobian_dot_;
    std::vector<double> jnt_pos_up_limits_;
    std::vector<double> jnt_pos_down_limits_;
    std::vector<double> jnt_vel_limits_;
};

}

#endif  // PREDIKCT_ROBOT_MODEL_H