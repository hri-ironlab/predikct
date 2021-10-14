/*
Defines a class for packaging the details of a robot state including position, velocity, and calculated values such as the Jacobian.

Author: Connor Brooks
*/

#ifndef PREDIKCT_MOTION_STATE_H
#define PREDIKCT_MOTION_STATE_H

//includes
#include <vector>
#include <Eigen/Core>
#include <kdl/frames.hpp>
#include <predikct/robot_model.h>
#include <cmath>

namespace predikct
{

class MotionState
{
public:
    // Constructor takes in a current joint position and the last velocity applied
    MotionState(std::vector<double>& last_joint_positions, std::vector<double>& last_joint_velocities, std::vector<double>& last_null_vector)
    {
        joint_positions = last_joint_positions;
        joint_velocities = last_joint_velocities;
        null_goals = last_null_vector;
        is_jacobian_calculated = false;
        is_position_calculated = false;
        time_in_future = 0.0;
    }

    // Constructor takes in a pointer to the starting joint positions, a set of old and new joint velocities and a timestamp over which to spin up toward and apply the new joint velocities
    MotionState(std::vector<double>* starting_joint_positions, std::vector<double>* last_joint_velocities, std::vector<double>* given_joint_velocities, std::vector<double>* null_vector, double motion_time, RobotModel* robot_model, double time_into_future)
    {
        double timestamp, time_increment, max_acceleration, current_velocity, current_position, target_velocity;
        time_increment = 0.01;
        max_acceleration = 0.25; //Fetch: 0.075
        time_in_future = time_into_future;
        for(int j = 0; j < (*starting_joint_positions).size(); j++)
        {
            timestamp = 0.0;
            double current_velocity = (*last_joint_velocities)[j];
            double current_position = (*starting_joint_positions)[j];
            double target_velocity = (*given_joint_velocities)[j];
            if(abs(target_velocity) > robot_model->GetJointVelLimit(j))
            {
                target_velocity = copysign(robot_model->GetJointVelLimit(j), target_velocity);
            }
            while(timestamp < motion_time)
            {
                current_position += current_velocity * time_increment;
                if(abs(target_velocity - current_velocity) > max_acceleration)
                {
                    current_velocity += copysign(max_acceleration, target_velocity - current_velocity);
                } else {
                    current_velocity = target_velocity;
                }
                timestamp += time_increment;
            }
            
            joint_velocities.push_back(current_velocity);
            joint_positions.push_back(current_position);
            commanded_velocities.push_back(target_velocity);
            null_goals.push_back((*null_vector)[j]);

            if(joint_positions[j] > 0 && joint_positions[j] > robot_model->GetJointPosUpLimit(j))
            {
                joint_positions[j] = robot_model->GetJointPosUpLimit(j);
            }
            else if(joint_positions[j] < 0 && abs(joint_positions[j]) > robot_model->GetJointPosDownLimit(j))
            {
                joint_positions[j] = -1 * robot_model->GetJointPosDownLimit(j);
            }
        }
        is_jacobian_calculated = false;
        is_position_calculated = false;
    }
    ~MotionState()
    {}

    void CalculateJacobian(RobotModel* robot_model)
    {
        if(is_jacobian_calculated)
        {
            return;
        }
        robot_model->GetJacobian(joint_positions, &jacobian);
        is_jacobian_calculated = true;
    }

    void CalculatePosition(RobotModel* robot_model)
    {
        if(is_position_calculated)
        {
            return;
        }
        robot_model->GetPosition(joint_positions, &position);
        is_position_calculated = true;
    }

    //Resulting positions after applying joint velocities over given time period
    std::vector<double> joint_positions;
    std::vector<double> joint_velocities;
    std::vector<double> commanded_velocities;
    std::vector<double> null_goals;
    bool is_jacobian_calculated;
    Eigen::Matrix<double,6,Eigen::Dynamic> jacobian;
    bool is_position_calculated;
    double time_in_future;
    KDL::Frame position;
};

}

#endif  // PREDIKCT_MOTION_STATE_H