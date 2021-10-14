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

//Calculates the Yoshikawa manipulability measure for singularity avoidance/closeness to singularities
double RewardCalculator::CalculateManipulability(boost::shared_ptr<MotionState> candidate_motion)
{
    Eigen::MatrixXd pseudo_inverse = candidate_motion->jacobian * candidate_motion->jacobian.transpose();
    return sqrt(pseudo_inverse.determinant());
}

//Returns the number of joints sufficiently close to their limit
double RewardCalculator::CalculateLimitCloseness(RobotModel* robot_model, boost::shared_ptr<MotionState> candidate_motion)
{
    double limit_closeness = 0.5; //Fetch: 0.2
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
double RewardCalculator::EvaluateMotionCandidate(RobotModel* robot_model, boost::shared_ptr<MotionState> old_state, boost::shared_ptr<MotionState> candidate_motion, KDL::Frame* ideal_position, bool verbose)
{
    candidate_motion->CalculatePosition(robot_model);
    candidate_motion->CalculateJacobian(robot_model);

    // Calculate distance between resulting position and position from idealized movement
    double x, y, z, w;
    candidate_motion->position.M.GetQuaternion(x, y, z, w);

    if(verbose)
    {
        ROS_ERROR("Motion Candidate resulting pose: %.2f %.2f %.2f %.2f %.2f %.2f %.2f", candidate_motion->position.p.x(), candidate_motion->position.p.y(), candidate_motion->position.p.z(), x, y, z, w);
        ROS_ERROR("Motion Candidate commanded velocities: %.2f %.2f %.2f %.2f %.2f %.2f %.2f", candidate_motion->commanded_velocities[0], candidate_motion->commanded_velocities[1], candidate_motion->commanded_velocities[2], candidate_motion->commanded_velocities[3], candidate_motion->commanded_velocities[4], candidate_motion->commanded_velocities[5], candidate_motion->commanded_velocities[6]);
        ROS_ERROR("Motion Candidate resulting velocities: %.2f %.2f %.2f %.2f %.2f %.2f %.2f", candidate_motion->joint_velocities[0], candidate_motion->joint_velocities[1], candidate_motion->joint_velocities[2], candidate_motion->joint_velocities[3], candidate_motion->joint_velocities[4], candidate_motion->joint_velocities[5], candidate_motion->joint_velocities[6]);
    }
    
    double distance_estimate = CalculateDistance(&(candidate_motion->position), ideal_position, 1.0, 1.0);

    // Calculate motion smoothness as determined by size of acceleration for each joint
    double acceleration_size = CalculateSmoothness(&(old_state->joint_velocities), &(candidate_motion->commanded_velocities));

    // Estimate closeness to singularities
    double manipulability = CalculateManipulability(candidate_motion);

    double num_limited_joints = CalculateLimitCloseness(robot_model, candidate_motion);

    if(verbose)
    {
        ROS_ERROR("Distance Score: %.2f", distance_estimate);
        ROS_ERROR("acceleration size: %.2f", acceleration_size);
        ROS_ERROR("manipulability: %.2f", manipulability);
        ROS_ERROR("individual weighted components: %.3f, %.3f, %.3f, %.3f", -10*distance_estimate, -1*acceleration_size, 500*manipulability, -1.0 * num_limited_joints);
    }

    //Fetch: -25 -1 100 -2
    //Panda: -150 -5 25 -7
    return (-150 * distance_estimate) + (-5 * acceleration_size) + (25 * manipulability) + (-20.0 * num_limited_joints);
}

}