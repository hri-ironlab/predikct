/*
Defines a base class for user models for use in Iron Lab's PrediKCT Controller.
Subclasses from this base class should implement specific types of probabilistic user models.

Author: Connor Brooks
*/

#ifndef PREDIKCT_USER_MODEL_H
#define PREDIKCT_USER_MODEL_H

//includes
#include <vector>
#include <random>
#include <boost/shared_ptr.hpp>

namespace predikct
{

// forward declares
class RobotModel;
class MotionState;

class UserModel
{
public:
    UserModel();
    
    ~UserModel()
    {}

    virtual void SampleNoReplacement(RobotModel* robot_model, boost::shared_ptr<MotionState> state, std::vector<double>* last_velocity_command, std::vector<std::vector<double>>* movement_options, 
        double movement_timestep, int num_samples, std::vector<std::vector<double>>* samples, std::vector<double>* sample_probabilities);

    void SampleProbabilitiesNoReplacement(std::vector<double>* probabilities, int num_samples, std::vector<int>* chosen_indices, std::vector<double>* chosen_probabilities);

private:
    std::random_device rd;
    std::default_random_engine random_generator;
    std::uniform_real_distribution<double> random_distribution;
};

}

#endif  // PREDIKCT_USER_MODEL_H
