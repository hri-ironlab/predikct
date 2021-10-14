/*
Defines a user model class for use in Iron Lab's Predictive Velocity Controller.

Author: Connor Brooks
*/

#ifndef PREDIKCT_WAYPOINT_USER_MODEL_H
#define PREDIKCT_WAYPOINT_USER_MODEL_H

//includes
#include <vector>
#include <random>
#include <boost/shared_ptr.hpp>
#include <kdl/frames.hpp>
#include "user_model.h"
#include "reward_calculator.h"

namespace predikct
{

// forward declares
class RobotModel;
class MotionState;

class WaypointUserModel : public UserModel
{
public:

    void SetNextWaypoint(double p_x, double p_y, double p_z, double q_x, double q_y, double q_z, double q_w, double seconds_till_active);

    void DecrementTime(double seconds);

    void SampleNoReplacement (RobotModel* robot_model, boost::shared_ptr<MotionState> state, std::vector<double>* last_velocity_command, std::vector<std::vector<double>>* movement_options, 
        double movement_timestep, int num_samples, std::vector<std::vector<double>>* samples, std::vector<double>* sample_probabilities) override;

private:
    KDL::Frame current_waypoint;
    KDL::Frame next_waypoint;
    double seconds_till_next;
};

}

#endif  // PREDIKCT_WAYPOINT_USER_MODEL_H