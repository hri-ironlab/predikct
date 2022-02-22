#include <ros/ros.h>
#include <vector>
#include <boost/format.hpp>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <string>
#include <std_msgs/String.h>
#include <sensor_msgs/JointState.h>
#include "predikct/robot_model.h"
#include "predikct/user_model.h"
#include "predikct/reward_calculator.h"
#include "predikct/motion_state.h"
#include "predikct/motion_candidate_node.h"

class Controller
{
public:
    Controller(ros::NodeHandle& nh)
    {
        // Define Subs & Pubs to receive current joint states and task space motion requests and publish joint velocity commands
        joint_sub = nh.subscribe<sensor_msgs::JointState>("joint_states", 1, boost::bind(&Controller::JointUpdateCallback, this, _1));
        vel_command_sub = nh.subscribe<geometry_msgs::Twist>("teleop_commands", 1, boost::bind(&Controller::VelocityCommandCallback, this, _1));
        command_pub = nh.advertise<sensor_msgs::JointState>("joint_commands", 1);

        // Set tree parameters. These can be defined as needed but should be tuned for your use case
        // Larger trees will typically provide smoother movement for a given time interval, but will require that time interval to be larger
        // Reward parameters can be configured to emphasize different components (or you can define a reward model that is entirely custom!)
        tree_spec.motion_candidate_branching_factor = 6;
        tree_spec.temporal_discount = 0.95;
        tree_spec.time_window = 0.5;
        tree_spec.tree_depth = 2;
        tree_spec.user_prediction_branching_factor = 16;
        tree_spec.velocity_primitive_set_size = 4 * tree_spec.user_prediction_branching_factor;
        double dist_weight = -50;
        double accel_weight = -1;
        double manip_weight = 500;
        double lim_weight = -2;
        reward.SetParameters(dist_weight, accel_weight, manip_weight, lim_weight);

        current_fetch_command_msg = new sensor_msgs::JointState();
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
    }

    ~Controller()
    {}

    void ReadParams()
    {
        int no_of_joints;
        ros::param::get("/KCT_Controller/number_of_joints", no_of_joints);
        joint_names.clear();
        std::string new_joint_name;
        double new_joint_limit;
        for(int i = 0; i < no_of_joints; ++i)
        {
            ros::param::get("/KCT_Controller/joint" + std::to_string(i) + "_name", new_joint_name);
            joint_names.push_back(new_joint_name);
        }
        user = new predikct::UserModel();
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
            // Bad message or message about joints other than the arm, discard
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
        boost::shared_ptr<predikct::MotionState> current_state( new predikct::MotionState(&last_joint_positions, &last_joint_velocities, last_velocity_msg, &last_null_vector, avg_search_time, &robot, 0.0) );
        
        double tree_start = ros::Time::now().toSec();
        predikct::MotionCandidateNode tree_root(nullptr, &robot, current_state, 1, &tree_spec, 
        &reward, user, &last_velocity_command);
        last_velocity_command = std::vector<double>(6, 0);
        tree_root.ChooseMotionCandidate(&(current_fetch_command_msg->velocity), &(last_null_vector));
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

    predikct::TreeSpec tree_spec;
    int loop_rate;

private:
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
    predikct::RobotModel robot;
    predikct::UserModel* user;
    predikct::RewardCalculator reward;
    bool joint_update_received;
    bool command_waiting;
    bool active;
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
