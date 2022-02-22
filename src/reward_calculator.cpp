/*
Most basic version of a reward calculator. Rewards smooth motion that properly applies velocity commands.
Subclasses of this class can be used for alternative versions of reward calculator.

Author: Connor Brooks
*/

//includes
#include "predikct/reward_calculator.h"
#include "predikct/motion_state.h"
#include <cmath>

namespace predikct
{

//--------------------------------------------------------------------------------------------------
// Utility functions

//Euclidean distance between two KDL Vectors
double RewardCalculator::GetLinearDistance(KDL::Vector* position_1, KDL::Vector* position_2)
{
    return sqrt(pow((position_1->data[0] - position_2->data[0]),2) + pow((position_1->data[1] - position_2->data[1]),2) 
        + pow((position_1->data[2] - position_2->data[2]),2));
}

//Distance between two KDL Rotations using 2 times arccos of dot product of the two quaternions
double RewardCalculator::GetAngularDistance(KDL::Rotation* rotation_1, KDL::Rotation* rotation_2)
{
    double rot1_x, rot1_y, rot1_z, rot1_w;
    rotation_1->GetQuaternion(rot1_x, rot1_y, rot1_z, rot1_w);
    double rot2_x, rot2_y, rot2_z, rot2_w;
    rotation_2->GetQuaternion(rot2_x, rot2_y, rot2_z, rot2_w);
    double dot_product = abs(rot1_x*rot2_x + rot1_y*rot2_y + rot1_z*rot2_z + rot1_w*rot2_w);
    if(dot_product > 1.0)
    {
        dot_product = 1.0;
    }
    return 2*acos(dot_product);
}

//--------------------------------------------------------------------------------------------------
// Primary Scoring Functions

double RewardCalculator::CalculateDistance(KDL::Frame* frame_1, KDL::Frame* frame_2, double linear_distance_weight, double angular_distance_weight)
{
    double linear_distance = GetLinearDistance(&(frame_1->p), &(frame_2->p));
    double angular_distance = GetAngularDistance(&(frame_1->M), &(frame_2->M));
    return linear_distance*linear_distance_weight + angular_distance*angular_distance_weight;
}

//Calculates smoothness as the L2 norm of the acceleration.
double RewardCalculator::CalculateSmoothness(std::vector<double>* old_velocities, std::vector<double>* new_velocities)
{
    double sum_squared_diffs = 0.0;
    for(int i = 0; i < new_velocities->size(); i++)
    {
        sum_squared_diffs += pow(((*new_velocities)[i] - (*old_velocities)[i]),2);
    }

    return sqrt(sum_squared_diffs);
}

//Retrieves the Yoshikawa manipulability measure for singularity avoidance/closeness to singularities
double RewardCalculator::CalculateManipulability(RobotModel* robot_model, boost::shared_ptr<MotionState> candidate_motion)
{
    candidate_motion->CalculateManipulability(robot_model);
    return candidate_motion->manipulability;
}

//Returns the number of joints sufficiently close to their limit
double RewardCalculator::CalculateLimitCloseness(RobotModel* robot_model, boost::shared_ptr<MotionState> candidate_motion)
{
    // TODO: Make this threshold a ros param
    double limit_closeness = 0.25;
    double num_limit_joints = 0.0;
    for(int i = 0; i < candidate_motion->joint_positions.size(); ++i)
    {
        if(robot_model->GetJointPosUpLimit(i) != std::numeric_limits<double>::infinity()){
            double min_dist = std::min(candidate_motion->joint_positions[i] - (-1 * robot_model->GetJointPosDownLimit(i)), robot_model->GetJointPosUpLimit(i) - candidate_motion->joint_positions[i]);
            if(min_dist < limit_closeness)
            {
                num_limit_joints += 1.0;
            }
        }
    }
    return num_limit_joints;
}

//Master function for determining the reward a possible motion candidate gets
// This version includes a weighted sum of:
// 1. Distance between "idealized" motion result and this motion result (using assumption robot speed: radians/sec ~= 4*meters/sec)
// 2, Motion smoothness
// 3. Closeness to singularities
// 4. Closeness to joint limits
double RewardCalculator::EvaluateMotionCandidate(RobotModel* robot_model, boost::shared_ptr<MotionState> old_state, boost::shared_ptr<MotionState> candidate_motion, KDL::Frame* ideal_position)
{
    candidate_motion->CalculatePosition(robot_model);
    candidate_motion->CalculateJacobian(robot_model);

    // Calculate distance between resulting position and position from idealized movement
    double x, y, z, w;
    candidate_motion->position.M.GetQuaternion(x, y, z, w);
    
    double distance_estimate = CalculateDistance(&(candidate_motion->position), ideal_position, 1.0, 1.0);

    // Calculate motion smoothness as determined by size of acceleration for each joint
    double acceleration_size = CalculateSmoothness(&(old_state->joint_velocities), &(candidate_motion->commanded_velocities));

    // Estimate closeness to singularities
    double manipulability = CalculateManipulability(robot_model, candidate_motion);

    double num_limited_joints = CalculateLimitCloseness(robot_model, candidate_motion);

    return (dist_weight * distance_estimate) + (accel_weight * acceleration_size) + (manip_weight * manipulability) + (lim_weight * num_limited_joints);
}

void RewardCalculator::SetParameters(double distance_weight, double acceleration_weight, double manipulability_weight, double limits_weight)
{
    dist_weight = distance_weight;
    accel_weight = acceleration_weight;
    manip_weight = manipulability_weight;
    lim_weight = limits_weight;
}

}