#include <ros/ros.h>
#include <vector>
#include <boost/format.hpp>
#include <boost/pointer_cast.hpp>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <string>
#include <std_msgs/String.h>
#include <sensor_msgs/JointState.h>
#include "predikct/tree_node.h"
#include "predikct/robot_model.h"
#include "predikct/user_model.h"
#include "predikct/reward_calculator.h"
#include "predikct/motion_state.h"
#include "predikct/motion_candidate_node.h"

class Controller
{
public:
    Controller(ros::NodeHandle& nh) : tree_spec(new predikct::TreeSpec), robot(new predikct::RobotModel),
    user(new predikct::UserModel), reward(new predikct::RewardCalculator)
    {
        ReadParams();
        joint_sub = nh.subscribe<sensor_msgs::JointState>("joint_states", 1, boost::bind(&Controller::JointUpdateCallback, this, _1));
        vel_command_sub = nh.subscribe<geometry_msgs::Twist>("teleop_commands", 1, boost::bind(&Controller::VelocityCommandCallback, this, _1));
        command_pub = nh.advertise<sensor_msgs::JointState>("joint_commands", 1);

        last_velocity_command = std::vector<double>(7, 0);
        last_joint_positions = std::vector<double>(7, 0);
        last_joint_velocities = std::vector<double>(7, 0);
        last_null_vector = std::vector<double>(7, 0);
        current_fetch_command_msg->velocity = std::vector<double>(last_joint_velocities.size(), 0);
        last_joint_msg_time = 0;
        avg_search_time = 0;
        search_time_window = std::vector<double>(10, 0);
        oldest_search_time = 0;
        command_timeout_time = 0.25;
        last_received_command_time = 0;
        ReadParams();
        active = false;
        ros::param::get("/KCT_Controller/verbose", verbose);
        if (verbose)
        {
            ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug);
        }
    }

    ~Controller()
    {}

    void ReadParams()
    {
        ros::param::get("/KCT_Controller/number_of_joints", no_of_joints);
        joint_names.clear();
        std::string new_joint_name;
        for(int i = 0; i < no_of_joints; ++i)
        {
            ros::param::get("/KCT_Controller/joint" + std::to_string(i) + "_name", new_joint_name);
            joint_names.push_back(new_joint_name);
        }
        user = boost::shared_ptr<predikct::UserModel>(new predikct::UserModel);
        ros::param::get("/KCT_Controller/tree_spec/depth", tree_spec->tree_depth);
        ros::param::get("/KCT_Controller/tree_spec/motion_candidate_branching_factor", tree_spec->motion_candidate_branching_factor);
        ros::param::get("/KCT_Controller/tree_spec/user_prediction_branching_factor", tree_spec->user_prediction_branching_factor);
        ros::param::get("/KCT_Controller/tree_spec/velocity_primitive_set_size", tree_spec->velocity_primitive_set_size);
        ros::param::get("/KCT_Controller/tree_spec/time_window", tree_spec->time_window);
        ros::param::get("/KCT_Controller/tree_spec/temporal_discount", tree_spec->temporal_discount);
        double dist_weight, accel_weight, manip_weight, lim_weight;
        ros::param::get("/KCT_Controller/reward_params/distance", dist_weight);
        ros::param::get("/KCT_Controller/reward_params/acceleration", accel_weight);
        ros::param::get("/KCT_Controller/reward_params/manipulability", manip_weight);
        ros::param::get("/KCT_Controller/reward_params/limits", lim_weight);
        reward->SetParameters(dist_weight, accel_weight, manip_weight, lim_weight);
    }

    void JointUpdateCallback(const sensor_msgs::JointState::ConstPtr& msg)
    {
        float k_min_update_time = 0.02;
        std::vector<double> new_joint_positions;
        std::vector<double> new_joint_velocities;
        double joint_msg_time = msg->header.stamp.now().toSec();
        if(joint_msg_time - last_joint_msg_time < k_min_update_time)
        {
            // Only update joint positions and velocities at some max rate to ensure proper estimates of joint velocities
            return;
        }
        int j = 0;
        for(int i = 0; i < msg->position.size(); ++i)
        {
            if(j < last_joint_positions.size() && msg->name[i].compare(joint_names[j]) == 0)
            {
                new_joint_positions.push_back(msg->position[i]);
                new_joint_velocities.push_back((new_joint_positions[j] - last_joint_positions[j]) / (joint_msg_time - last_joint_msg_time));
                ++j;
            }
        }
        if(new_joint_positions.size() != joint_names.size())
        {
            return;
        }
        last_joint_positions = new_joint_positions;
        last_joint_velocities = new_joint_velocities;
        last_joint_msg_time = joint_msg_time;
    }

    void VelocityCommandCallback(const geometry_msgs::Twist::ConstPtr& msg)
    {
        if(active)
        {
            return;
        }
        std::vector<double> new_velocity_command;
        new_velocity_command.push_back(msg->linear.x);
        new_velocity_command.push_back(msg->linear.y);
        new_velocity_command.push_back(msg->linear.z);
        new_velocity_command.push_back(msg->angular.x);
        new_velocity_command.push_back(msg->angular.y);
        new_velocity_command.push_back(msg->angular.z);
        last_velocity_command = new_velocity_command;
        last_received_command_time = ros::Time::now().toSec();
    }

    void GetNewCommand()
    {
        active = true;
        bool all_zeros = true;
        for(int i = 0; i < last_velocity_command.size(); ++i)
        {
            if(last_velocity_command[i] != 0.0)
            {
                all_zeros = false;
                break;
            }
        }
        if(all_zeros || (ros::Time::now().toSec() - last_received_command_time) > command_timeout_time)
        {
            active = false;
            PublishStop();
            return;
        }

        std::vector<double>* last_velocity_msg;
        last_velocity_msg = &(current_fetch_command_msg->velocity);
        boost::shared_ptr<predikct::MotionState> current_state( new predikct::MotionState(&last_joint_positions, &last_joint_velocities, last_velocity_msg, &last_null_vector, avg_search_time, robot, 0.0) );
        
        double tree_start = ros::Time::now().toSec();
        boost::shared_ptr<predikct::MotionCandidateNode> tree_root(new predikct::MotionCandidateNode(boost::weak_ptr<predikct::TreeNode>(), robot, current_state, 1, tree_spec, reward, user, &last_velocity_command, verbose));
        last_velocity_command = std::vector<double>(6, 0);
        tree_root->ChooseMotionCandidate(&(current_fetch_command_msg->velocity), &(last_null_vector));
        double tree_time = ros::Time::now().toSec() - tree_start;
        avg_search_time = (avg_search_time*search_time_window.size() + tree_time - search_time_window[oldest_search_time]) / search_time_window.size();
        search_time_window[oldest_search_time] = tree_time;
        if(++oldest_search_time >= search_time_window.size())
        {
            oldest_search_time = 0;
        }

        active = false;
        PublishCommand();
    }

    void PublishCommand()
    {
        command_pub.publish(*(current_fetch_command_msg));
    }

    void PublishStop()
    {
        current_fetch_command_msg->velocity = std::vector<double>(last_joint_velocities.size(), 0);
        PublishCommand();
    }

    boost::shared_ptr<predikct::TreeSpec> tree_spec;
    int loop_rate;

private:
    int no_of_joints;
    ros::Subscriber joint_sub;
    ros::Subscriber vel_command_sub;
    ros::Publisher command_pub;
    std::vector<double> last_joint_positions;
    std::vector<double> last_joint_velocities;
    std::vector<double> last_null_vector;
    double last_joint_msg_time;
    std::vector<double> last_velocity_command;
    std::vector<std::string> joint_names;
    std::vector<double> joint_vel_limits;
    boost::shared_ptr<predikct::RobotModel> robot;
    boost::shared_ptr<predikct::UserModel> user;
    boost::shared_ptr<predikct::RewardCalculator> reward;
    bool joint_update_received;
    bool command_waiting;
    bool active;
    bool verbose;
    sensor_msgs::JointState* current_fetch_command_msg;
    double avg_search_time;
    std::vector<double> search_time_window;
    int oldest_search_time;
    double last_received_command_time;
    double command_timeout_time;
};

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "KCT_Controller");

    ros::NodeHandle nh;
    ros::Duration(1.0).sleep();

    Controller controller(nh);
    ros::Duration(1.0).sleep();
    ros::Rate update_loop_rate(100);
    while(ros::ok())
    {
        ros::spinOnce();
        controller.GetNewCommand();
        update_loop_rate.sleep();
    }
}
