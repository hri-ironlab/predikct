/*
Defines a user model class for use in Iron Lab's Predictive Velocity Controller.

Author: Connor Brooks
*/

#ifndef PREDIKCT_RANDOM_USER_MODEL_H
#define PREDIKCT_RANDOM_USER_MODEL_H

//includes
#include <vector>
#include <random>
#include <boost/shared_ptr.hpp>
#include "user_model.h"

namespace predikct
{

// forward declares
class RobotModel;
class MotionState;

class RandomUserModel : public UserModel
{
public:

    void SampleNoReplacement (RobotModel* robot_model, boost::shared_ptr<MotionState> state, std::vector<double>* last_velocity_command, std::vector<std::vector<double>>* movement_options, 
        double movement_timestep, int num_samples, std::vector<std::vector<double>>* samples, std::vector<double>* sample_probabilities) override;
};

}

#endif  // PREDIKCT_RANDOM_USER_MODEL_H