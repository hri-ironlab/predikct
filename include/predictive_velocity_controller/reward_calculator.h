/*
Defines a base class for reward calculators for use in Iron Lab's Predictive Velocity Controller.
Subclasses from this base class should implement specific types of reward calculations.

Author: Connor Brooks
*/

#ifndef PREDIKCT_REWARD_CALCULATOR_H
#define PREDIKCT_REWARD_CALCULATOR_H

//includes
#include <vector>
#include <kdl/frames.hpp>
#include <Eigen/LU>
#include <boost/shared_ptr.hpp>

namespace predikct
{

// forward declares
class RobotModel;
class MotionState;

class RewardCalculator
{
public:
    RewardCalculator()
    {
        
    }
    ~RewardCalculator()
    {}

    static double GetLinearDistance(KDL::Vector* position_1, KDL::Vector* position_2);

    static double GetAngularDistance(KDL::Rotation* rotation_1, KDL::Rotation* rotation_2);

    static double CalculateDistance(KDL::Frame* frame_1, KDL::Frame* frame_2, double linear_distance_weight, double angular_distance_weight);

    static double CalculateSmoothness(std::vector<double>* old_velocities, std::vector<double>* new_velocities);

    static double CalculateManipulability(boost::shared_ptr<MotionState> candidate_motion);

    static double CalculateLimitCloseness(RobotModel* robot_model, boost::shared_ptr<MotionState> candidate_motion);

    double EvaluateMotionCandidate(RobotModel* robot_model, boost::shared_ptr<MotionState> old_state, boost::shared_ptr<MotionState> candidate_motion, KDL::Frame* ideal_position, bool verbose);

    double GetDistance(KDL::Frame* frame_1, KDL::Frame* frame_2);
};

}

#endif  // PREDIKCT_REWARD_CALCULATOR_H