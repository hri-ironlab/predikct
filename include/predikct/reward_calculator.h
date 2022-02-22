/*
Defines a base class for reward calculators for use in Iron Lab's PrediKCT controller.
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
        dist_weight = -1.0;
        accel_weight = -1.0;
        manip_weight = 1.0;
        lim_weight = -1.0;
    }
    ~RewardCalculator()
    {}

    static double GetLinearDistance(KDL::Vector* position_1, KDL::Vector* position_2);

    static double GetAngularDistance(KDL::Rotation* rotation_1, KDL::Rotation* rotation_2);

    static double CalculateDistance(KDL::Frame* frame_1, KDL::Frame* frame_2, double linear_distance_weight, double angular_distance_weight);

    static double CalculateSmoothness(std::vector<double>* old_velocities, std::vector<double>* new_velocities);

    static double CalculateManipulability(RobotModel* robot_model, boost::shared_ptr<MotionState> candidate_motion);

    static double CalculateLimitCloseness(RobotModel* robot_model, boost::shared_ptr<MotionState> candidate_motion);

    double EvaluateMotionCandidate(RobotModel* robot_model, boost::shared_ptr<MotionState> old_state, boost::shared_ptr<MotionState> candidate_motion, KDL::Frame* ideal_position);

    double GetDistance(KDL::Frame* frame_1, KDL::Frame* frame_2);

    void SetParameters(double distance_weight, double acceleration_weight, double manipulability_weight, double limits_weight);

    double GetDistWeight(){ return dist_weight; }
    double GetAccelWeight(){ return accel_weight; }
    double GetManipWeight(){ return manip_weight; }
    double GetLimWeight(){ return lim_weight; }

private:
    double dist_weight;
    double accel_weight;
    double manip_weight;
    double lim_weight;
};

}

#endif  // PREDICTIVE_VELOCITY_CONTROLLER_REWARD_CALCULATOR_H
