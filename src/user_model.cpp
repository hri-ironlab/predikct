//Base version of user model. This class implements a user model that models the user as exponentially more likely to choose velocities the closer they are to the last velocity choice.
//Subclasses of this user model class should implement other strategies for predicting user movement selection.

//includes
#include "predikct/user_model.h"
#include <cmath>
#include <ros/console.h>

namespace predikct
{

//----------------------------------------------------------------------------------------------------------------------------
// Utility functions for this version of user model

//Calculates velocity difference as the L2 norm of the acceleration going from one velocity to the other
double CalculateVelocityDiff(std::vector<double>* old_velocities, std::vector<double>* new_velocities)
{
    double sum_squared_diffs = 0.0;
    for(int i = 0; i < new_velocities->size(); i++)
    {
        sum_squared_diffs += pow(((*new_velocities)[i] - (*old_velocities)[i]),2);
    }

    return std::max(sqrt(sum_squared_diffs), 0.1);
}

//----------------------------------------------------------------------------------------------------------------------------
// UserModel definition

UserModel::UserModel() : random_generator(rd()), random_distribution(0.0, 1.0)
{ }

void UserModel::SampleNoReplacement(boost::shared_ptr<RobotModel> robot_model, boost::shared_ptr<MotionState> state, std::vector<double>* last_velocity_command,
    std::vector<std::vector<double>>* movement_options, double movement_timestep, int num_samples, std::vector<std::vector<double>>* samples, 
    std::vector<double>* sample_probabilities)
{
    // Generate scores according to distance from last velocity command
    double score_norm = 0.0;
    std::vector<double> candidate_scores;
    for(int i = 0; i < movement_options->size(); ++i)
    {
        double new_score = exp(1.0 / CalculateVelocityDiff(last_velocity_command, &(*movement_options)[i]));
        candidate_scores.push_back(new_score);
        score_norm += new_score;
    }

    std::vector<double> probabilities;
    for(int i = 0; i < movement_options->size(); ++i)
    {
        probabilities.push_back(candidate_scores[i] / score_norm);
    }
    
    // Sample with higher scores exponentially more likely to be chosen
    std::vector<int> chosen_samples;
    std::vector<double> chosen_probabilities;
    SampleProbabilitiesNoReplacement(&probabilities, num_samples, &chosen_samples, &chosen_probabilities);
    samples->clear();
    sample_probabilities->clear();
    for(int i = 0; i < chosen_samples.size(); ++i)
    {
        samples->push_back((*movement_options)[chosen_samples[i]]);
        sample_probabilities->push_back(chosen_probabilities[i]);
    }
}

// Returns an array of ints corresponding to the chosen indices from the probability list
void UserModel::SampleProbabilitiesNoReplacement(std::vector<double>* probabilities, int num_samples, std::vector<int>* chosen_indices, std::vector<double>* chosen_probabilities)
{
    chosen_indices->clear();
    chosen_probabilities->clear();
    double chosen_probability_norm = 0.0;
    std::vector<double> renormalized_probabilities(probabilities->size(), 0);
    for(int i = 0; i < renormalized_probabilities.size(); ++i){
        renormalized_probabilities[i] = (*probabilities)[i];
    }
    if(num_samples >= probabilities->size())
    {
        for(int i = 0; i < probabilities->size(); ++i)
        {
            chosen_indices->push_back(i);
            chosen_probabilities->push_back((*probabilities)[i]);
            chosen_probability_norm += (*chosen_probabilities)[i];
        }
        for(int i = 0; i < chosen_probabilities->size(); ++i)
        {
            (*chosen_probabilities)[i] = (*chosen_probabilities)[i] / chosen_probability_norm;
        }
        return;
    }

    double random_result;
    double new_norm = 1.0;
    for(int i = 0; i < num_samples; ++i)
    {
        random_result = random_distribution(random_generator);
        for(int j = 0; j < renormalized_probabilities.size(); ++j)
        {
            random_result -= renormalized_probabilities[j];
            if(random_result <= 0)
            {
                chosen_indices->push_back(j);
                chosen_probabilities->push_back((*probabilities)[j]);
                chosen_probability_norm += (*probabilities)[j];
                new_norm -= renormalized_probabilities[j];
                renormalized_probabilities[j] = 0.0;
                break;
            }
        }

        if(random_result > 0)
        {
            ROS_ERROR("Sum of probabilities of all choices is less than 1. Remaining random result: %.4f", random_result);
            throw;
        }

        // Renormalize probabilities
        for(int j = 0; j < probabilities->size(); ++j)
        {
            renormalized_probabilities[j] = renormalized_probabilities[j] / new_norm;
        }
        new_norm = 1.0;
    }

    // Normalize probabilities of chosen set
    for(int i = 0; i < chosen_probabilities->size(); ++i)
    {
        (*chosen_probabilities)[i] = (*chosen_probabilities)[i] / chosen_probability_norm;
    }
}

}