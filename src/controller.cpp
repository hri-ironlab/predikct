#include <ros/ros.h>
#include <vector>
#include <boost/format.hpp>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <franka_core_msgs/JointCommand.h>
#include <string>
#include <std_msgs/String.h>
#include <sensor_msgs/JointState.h>
#include "predikct/robot_model.h"
#include "predikct/user_model.h"
#include "predikct/random_user_model.h"
#include "predikct/waypoint_user_model.h"
#include "predikct/reward_calculator.h"
#include "predikct/motion_state.h"
#include "predikct/motion_candidate_node.h"

class Controller
{
public:
    Controller(ros::NodeHandle& nh)
    {
        ros::param::get("/KCT_Controller/robot_type", robot_type);

        joint_sub = nh.subscribe<sensor_msgs::JointState>("joint_states", 1, boost::bind(&Controller::JointUpdateCallback, this, _1));
        vel_command_sub = nh.subscribe<geometry_msgs::Twist>("teleop_commands", 1, boost::bind(&Controller::VelocityCommandCallback, this, _1));
        waypoint_sub = nh.subscribe<geometry_msgs::PoseStamped>("waypoint_updates", 1, boost::bind(&Controller::WaypointCallback, this, _1));
        controller_info_sub = nh.subscribe<std_msgs::String>("tester_requests", 1, boost::bind(&Controller::InfoCallback, this, _1));
        controller_info_pub = nh.advertise<std_msgs::String>("send_controller_info", 1);
        if(robot_type == 0){
            command_pub = nh.advertise<sensor_msgs::JointState>("joint_commands", 1);
        } else {
            command_pub = nh.advertise<franka_core_msgs::JointCommand>("joint_commands", 1);
        }
        current_fetch_command_msg = new sensor_msgs::JointState();
        next_fetch_command_msg = new sensor_msgs::JointState();
        current_panda_command_msg = new franka_core_msgs::JointCommand();
        next_panda_command_msg = new franka_core_msgs::JointCommand();

        loop_rate = 5;
        next_command_time = 0;

        tree_spec.motion_candidate_branching_factor = 3;
        tree_spec.temporal_discount = 0.9;
        tree_spec.time_window = 0.7; //Fetch: 1.0
        tree_spec.tree_depth = 4;
        tree_spec.user_prediction_branching_factor = 2;
        tree_spec.velocity_primitive_set_size = 30;

        last_velocity_command = std::vector<double>(7, 0);
        last_joint_positions = std::vector<double>(7, 0);
        last_joint_velocities = std::vector<double>(7, 0);
        last_null_vector = std::vector<double>(7, 0);
        current_fetch_command_msg->velocity = std::vector<double>(last_joint_velocities.size(), 0);
        next_fetch_command_msg->velocity = std::vector<double>(last_joint_velocities.size(), 0);
        current_panda_command_msg->velocity = std::vector<double>(last_joint_velocities.size(), 0);
        next_panda_command_msg->velocity = std::vector<double>(last_joint_velocities.size(), 0);
        current_panda_command_msg->mode = franka_core_msgs::JointCommand::VELOCITY_MODE;
        next_panda_command_msg->mode = franka_core_msgs::JointCommand::VELOCITY_MODE;
        last_joint_msg_time = 0;
        joint_update_received = false;
        active = false;
        
        baseline = false;
        verbose = false;
        last_all_zeros = false;
        ReadParams();
        current_panda_command_msg->names = joint_names;
        next_panda_command_msg->names = joint_names;
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

        int user_model_type;
        ros::param::get("/KCT_Controller/user_model", user_model_type);
        if(user_model_type == 0)
        {
            user = new predikct::UserModel();
        }
        else if (user_model_type == 1)
        {
            user = new predikct::RandomUserModel();
        }
        else if (user_model_type == 2)
        {
            user = new predikct::WaypointUserModel();
            update_waypoints = true;
        }

        if(!baseline)
        {
            controller_spec = (boost::format("Controller using User Model %d with MCBF: %d, TD: %.2f, Time Window: %.2f, Depth: %d, UPBF: %d, and VPSS: %d, Updating Waypoints: %d") % user_model_type % tree_spec.motion_candidate_branching_factor % tree_spec.temporal_discount % tree_spec.time_window % tree_spec.tree_depth % tree_spec.user_prediction_branching_factor % tree_spec.velocity_primitive_set_size % update_waypoints).str();
        } else {
            controller_spec = "BASELINE";
            ROS_ERROR("Creating BASELINE controller");
        }
    }

    void JointUpdateCallback(const sensor_msgs::JointState::ConstPtr& msg)
    {
        std::vector<double> new_joint_positions;
        std::vector<double> new_joint_velocities;
        double joint_msg_time = msg->header.stamp.now().toSec();
        if(joint_msg_time - last_joint_msg_time < (1.0 / (10.0 * loop_rate)))
        {
            // Only update joint positions and velocities at a rate ten times that of control loop
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
            // Bad message, discard
            ROS_ERROR("WARNING: Improperly sized joint positions (missing joint names)");
            return;
        }
        last_joint_positions = new_joint_positions;
        last_joint_velocities = new_joint_velocities;
        last_joint_msg_time = joint_msg_time;
        joint_update_received = true;
    }

    void VelocityCommandCallback(const geometry_msgs::Twist::ConstPtr& msg)
    {
        std::vector<double> new_velocity_command;
        new_velocity_command.push_back(msg->linear.x);
        new_velocity_command.push_back(msg->linear.y);
        new_velocity_command.push_back(msg->linear.z);
        new_velocity_command.push_back(msg->angular.x);
        new_velocity_command.push_back(msg->angular.y);
        new_velocity_command.push_back(msg->angular.z);
        last_velocity_command = new_velocity_command;
        GetNewCommand();
    }

    void WaypointCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
    {
        if(!update_waypoints){
            return;
        }
        static_cast<predikct::WaypointUserModel*>(user)->SetNextWaypoint(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z, msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z, msg->pose.orientation.w, msg->header.stamp.toSec());
    }

    void InfoCallback(const std_msgs::String::ConstPtr& msg)
    {
        std::string delimiter = "/";
        std::string in_msg(msg->data);
        std::string command = in_msg.substr(0, in_msg.find(delimiter));
        in_msg.erase(0, in_msg.find(delimiter) + delimiter.length());
        if(command.compare("request_info") == 0){
            std_msgs::String controller_msg;
            controller_msg.data = controller_spec;
            controller_info_pub.publish(controller_msg);
        } else if (command.compare("update_tree") == 0){
            // Format: update_tree/motion candidate branching factor/user prediction branching factor/tree depth
            int mcbf = std::stoi(in_msg.substr(0, in_msg.find(delimiter)));
            tree_spec.motion_candidate_branching_factor = mcbf;
            in_msg.erase(0, in_msg.find(delimiter) + delimiter.length());
            int upbf = std::stoi(in_msg.substr(0, in_msg.find(delimiter)));
            tree_spec.user_prediction_branching_factor = upbf;
            in_msg.erase(0, in_msg.find(delimiter) + delimiter.length());
            int td = std::stoi(in_msg.substr(0, in_msg.find(delimiter)));
            tree_spec.tree_depth = td;
            ROS_ERROR("Setting tree specs to -- mcbf: %d, upbf: %d, depth: %d", tree_spec.motion_candidate_branching_factor, tree_spec.user_prediction_branching_factor, tree_spec.tree_depth);
        } else if (command.compare("new_task") == 0){
            last_velocity_command = std::vector<double>(6, 0);
            last_null_vector = std::vector<double>(7, 0);
        }
        
    }

    void GetNewCommand()
    {
        double tree_start = ros::Time::now().toSec();
        active = true;
        if(command_waiting)
        {
            if(robot_type == 0){
                current_fetch_command_msg = next_fetch_command_msg;
            } else {
                current_panda_command_msg = next_panda_command_msg;
            }
            command_waiting = false;
            PublishCommand();
        }
        next_command_time = tree_start + (1.0 / loop_rate);
        if (update_waypoints)
        {
            static_cast<predikct::WaypointUserModel*>(user)->DecrementTime(1.0 / loop_rate);
        }
        if(!joint_update_received)
        {
            if(robot_type == 0){
                next_fetch_command_msg->velocity = std::vector<double>(last_joint_velocities.size(), 0);
            } else {
                next_panda_command_msg->velocity = std::vector<double>(last_joint_velocities.size(), 0);
            }
            
            return;
        }
        bool all_zeros = true;
        for(int i = 0; i < last_velocity_command.size(); ++i)
        {
            if(last_velocity_command[i] != 0.0)
            {
                all_zeros = false;
                break;
            }
        }
        if(all_zeros)
        {
            if(robot_type == 0){
                next_fetch_command_msg->velocity = std::vector<double>(last_joint_velocities.size(), 0);
            } else {
                next_panda_command_msg->velocity = std::vector<double>(last_joint_velocities.size(), 0);
            }
            return;
        }
        if(verbose)
        {
            ROS_ERROR("Target velocity: %.2f %.2f %.2f / %.2f %.2f %.2f", last_velocity_command[0], last_velocity_command[1], last_velocity_command[2], last_velocity_command[3], last_velocity_command[4], last_velocity_command[5]);
            ROS_ERROR("Creating tree at time: %.2f", ros::Time::now().toSec());
        }

        std::vector<double>* last_velocity_msg;
        if(robot_type == 0){
            last_velocity_msg = &(current_fetch_command_msg->velocity);
        } else {
            last_velocity_msg = &(current_panda_command_msg->velocity);
        }
        
        boost::shared_ptr<predikct::MotionState> current_state( new predikct::MotionState(&last_joint_positions, &last_joint_velocities, last_velocity_msg, &last_null_vector, 1.0 / loop_rate, &robot, 0.0) );
        
        if(verbose)
        {
            ROS_ERROR("last actual jvels: %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f", last_joint_velocities[0], last_joint_velocities[1], last_joint_velocities[2], last_joint_velocities[3], last_joint_velocities[4], last_joint_velocities[5], last_joint_velocities[6]);
            ROS_ERROR("last commanded jvels: %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f", current_state->commanded_velocities[0], current_state->commanded_velocities[1], current_state->commanded_velocities[2], current_state->commanded_velocities[3], current_state->commanded_velocities[4], current_state->commanded_velocities[5], current_state->commanded_velocities[6]);
            ROS_ERROR("predicted next jvels: %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f", current_state->joint_velocities[0], current_state->joint_velocities[1], current_state->joint_velocities[2], current_state->joint_velocities[3], current_state->joint_velocities[4], current_state->joint_velocities[5], current_state->joint_velocities[6]);
            ROS_ERROR("last actual jposition: %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f", last_joint_positions[0], last_joint_positions[1], last_joint_positions[2], last_joint_positions[3], last_joint_positions[4], last_joint_positions[5], last_joint_positions[6]);
            ROS_ERROR("predicted jposition: %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f", current_state->joint_positions[0], current_state->joint_positions[1], current_state->joint_positions[2], current_state->joint_positions[3], current_state->joint_positions[4], current_state->joint_positions[5], current_state->joint_positions[6]);
            current_state->CalculatePosition(&robot);
            double x, y, z, w;
            current_state->position.M.GetQuaternion(x, y, z, w);
            ROS_ERROR("prediction next pose: %.2f %.2f %.2f / %.2f %.2f %.2f %.2f", current_state->position.p.x(), current_state->position.p.y(), current_state->position.p.z(), x, y, z, w);

        }
        predikct::MotionCandidateNode tree_root(nullptr, &robot, current_state, 1, &tree_spec, 
        &reward, user, &last_velocity_command);

        last_velocity_command = std::vector<double>(6, 0);

        if(!baseline)
        {
            if(robot_type == 0){
                tree_root.ChooseMotionCandidate(&(next_fetch_command_msg->velocity), &(last_null_vector));
            } else {
                tree_root.ChooseMotionCandidate(&(next_panda_command_msg->velocity), &(last_null_vector));
            }
        } else {
            if(robot_type == 0){
                tree_root.GetBaseline(&(next_fetch_command_msg->velocity));
            } else {
                tree_root.GetBaseline(&(next_panda_command_msg->velocity));
            }
        }
        command_waiting = true;

        float tree_end = ros::Time::now().toSec();
        if(verbose) {
            ROS_ERROR("Tree took: %.3f", tree_end - tree_start);
        }
        if(tree_end - tree_start > (1.0 / loop_rate))
        {
            ROS_ERROR("WARNING: Tree too large. Max time: %.3f, Actual time: %.3f", (1.0 / loop_rate), tree_end - tree_start);
        }
        // ROS_ERROR("Finished tree search at time: %.2f", ros::Time::now().toSec());
        // ROS_ERROR("COMMANDED joint velocities: %.2f %.2f %.2f %.2f %.2f %.2f %.2f", command_msg->velocity[0], command_msg->velocity[1], command_msg->velocity[2], command_msg->velocity[3], command_msg->velocity[4], command_msg->velocity[5], command_msg->velocity[6]);
    }

    void PublishCommand()
    {
        double time_now = ros::Time::now().toSec();
        if(command_waiting && time_now > next_command_time)
        {
            if(robot_type == 0){
                current_fetch_command_msg = next_fetch_command_msg;
            } else {
                current_panda_command_msg = next_panda_command_msg;
            }
            
            command_waiting = false;
        } else if (active && time_now - next_command_time > 2.0 / loop_rate)
        {
            active = false;
            if(robot_type == 0){
                current_fetch_command_msg->velocity = std::vector<double>(last_joint_velocities.size(), 0);
            } else {
                current_panda_command_msg->velocity = std::vector<double>(last_joint_velocities.size(), 0);
            }
            
        }

        if(robot_type == 0){
            command_pub.publish(*(current_fetch_command_msg));
        } else {
            if(current_panda_command_msg->velocity[0] == 0.0 && current_panda_command_msg->velocity[1] == 0.0 && current_panda_command_msg->velocity[2] == 0.0 && current_panda_command_msg->velocity[3] == 0.0 && current_panda_command_msg->velocity[4] == 0.0 && current_panda_command_msg->velocity[5] == 0.0 && current_panda_command_msg->velocity[6] == 0.0)
            {
                // Don't publish 0's to Panda repeatedly so that it can be reset. Controller timeout is extremely rapid anyway so movement will stop quickly
                if(last_all_zeros)
                {
                    return;
                } else {
                    last_all_zeros = true;
                }
            } else {
                last_all_zeros = false;
            }
            command_pub.publish(*(current_panda_command_msg));
        }
    }

    predikct::TreeSpec tree_spec;
    int loop_rate;

private:
    int robot_type;
    ros::Subscriber joint_sub;
    ros::Subscriber vel_command_sub;
    ros::Subscriber waypoint_sub;
    ros::Subscriber controller_info_sub;
    ros::Publisher controller_info_pub;
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
    bool update_waypoints;
    bool baseline;
    bool verbose;
    bool command_waiting;
    bool active;
    bool last_all_zeros;
    std::string controller_spec;
    sensor_msgs::JointState* current_fetch_command_msg;
    sensor_msgs::JointState* next_fetch_command_msg;
    franka_core_msgs::JointCommand* current_panda_command_msg;
    franka_core_msgs::JointCommand* next_panda_command_msg;
    double next_command_time;
};

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "KCT_Controller");

    ros::NodeHandle nh;
    ros::Duration(1.0).sleep();

    Controller controller(nh);
    ros::Duration(1.0).sleep();
    ros::Rate update_loop_rate(controller.loop_rate * 30);

    while(ros::ok())
    {
        ros::spinOnce();
        controller.PublishCommand();
        update_loop_rate.sleep();
    }
}