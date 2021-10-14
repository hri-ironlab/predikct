//User model that just assigns uniform probability to all possible movements. 

//includes
#include "predikct/random_user_model.h"
#include <cmath>
#include <ros/console.h>

namespace predikct
{

//----------------------------------------------------------------------------------------------------------------------------
// RandomUserModel definition

void RandomUserModel::SampleNoReplacement(RobotModel* robot_model, boost::shared_ptr<MotionState> state, std::vector<double>* last_velocity_command,
    std::vector<std::vector<double>>* movement_options, double movement_timestep, int num_samples, std::vector<std::vector<double>>* samples, 
    std::vector<double>* sample_probabilities)
{
    // Generate scores according to distance from last velocity command
    double score_norm = 0.0;
    double item_score = 1.0 / movement_options->size();
    std::vector<double> probabilities;
    std::vector<double> candidate_scores;
    for(int i = 0; i < movement_options->size(); ++i)
    {
        candidate_scores.push_back(item_score);
        probabilities.push_back(item_score);
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

}