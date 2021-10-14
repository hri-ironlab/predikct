//User model that just assigns probability based on distance to given waypoint pose

//includes
#include "predikct/waypoint_user_model.h"
#include "predikct/motion_state.h"
#include <cmath>
#include <ros/console.h>

namespace predikct
{

//Forward declares
class RewardCalculator;

//----------------------------------------------------------------------------------------------------------------------------
// WaypointUserModel definition

void WaypointUserModel::SetNextWaypoint(double p_x, double p_y, double p_z, double q_x, double q_y, double q_z, double q_w, double seconds_till_active)
{
    if(seconds_till_active == 0.0)
    {
        current_waypoint = KDL::Frame( KDL::Rotation::Quaternion(q_x, q_y, q_z, q_w), 
        KDL::Vector(p_x, p_y, p_z) );
    } else {
        next_waypoint = KDL::Frame( KDL::Rotation::Quaternion(q_x, q_y, q_z, q_w), 
            KDL::Vector(p_x, p_y, p_z) );
        seconds_till_next = seconds_till_active;
    }
}

void WaypointUserModel::DecrementTime(double seconds)
{
    seconds_till_next -= seconds;
    if(seconds_till_next <= 0) {
        current_waypoint = next_waypoint;
    }
}

void WaypointUserModel::SampleNoReplacement(RobotModel* robot_model, boost::shared_ptr<MotionState> state, std::vector<double>* last_velocity_command,
    std::vector<std::vector<double>>* movement_options, double movement_timestep, int num_samples, std::vector<std::vector<double>>* samples, 
    std::vector<double>* sample_probabilities)
{
    // Generate scores according to distance from current waypoint
    double score_norm = 0.0;
    std::vector<double> candidate_scores;
    state->CalculatePosition(robot_model);
    double x, y, z, w;
    state->position.M.GetQuaternion(x, y, z, w);
    current_waypoint.M.GetQuaternion(x, y, z, w);
    if(seconds_till_next > 0 && state->time_in_future > seconds_till_next){
        next_waypoint.M.GetQuaternion(x, y, z, w);
    }

    bool use_future_waypoint = seconds_till_next > 0 && state->time_in_future > seconds_till_next;
    double dist_to_start;
    if(use_future_waypoint) {
        dist_to_start = RewardCalculator::CalculateDistance(&(state->position), &next_waypoint, 1.0, 1.0);
    } else {
        dist_to_start = RewardCalculator::CalculateDistance(&(state->position), &current_waypoint, 1.0, 1.0);
    }
    for(int i = 0; i < movement_options->size(); ++i)
    {
        KDL::Frame resulting_position( KDL::Rotation::RPY((*movement_options)[i][3]*movement_timestep, (*movement_options)[i][4]*movement_timestep, 
            (*movement_options)[i][5]*movement_timestep) * state->position.M, 
            state->position.p + KDL::Vector((*movement_options)[i][0]*movement_timestep, (*movement_options)[i][1]*movement_timestep, (*movement_options)[i][2]*movement_timestep));
        double new_dist;
        if(use_future_waypoint)
        {
            new_dist = RewardCalculator::CalculateDistance(&resulting_position, &next_waypoint, 1.0, 1.0);
        } else
        {
            new_dist = RewardCalculator::CalculateDistance(&resulting_position, &current_waypoint, 1.0, 1.0);
        }
        double new_score = exp(20 * (dist_to_start - new_dist));
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

}