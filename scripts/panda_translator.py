#!/usr/bin/env python

import rospy
from future.utils import viewitems # for python2&3 efficient compatibility

from franka_interface import ArmInterface
from franka_dataflow.getch import getch
from franka_core_msgs.msg import JointCommand

limb = None
target_position = [0,0,0,0,0,0,0]
last_ideal_times = [0,0,0,0,0,0,0]
ideal_position = [0,0,0,0,0,0,0]
target_velocity = [0,0,0,0,0,0,0]
active_joints = []
new_position_command = False
last_velocity_time = 0
timeout_length = 0.2

def command_processor(cmd_msg):
    global target_position
    global target_velocity
    global new_position_command
    global last_velocity_time
    global ideal_position

    if(cmd_msg.mode == 1):
        target_position = cmd_msg.position
        new_position_command = True
    elif(cmd_msg.mode == 2):
        angles = limb.joint_angles()
        new_time = rospy.Time.now().to_sec()
        for v in range(len(cmd_msg.velocity)):
            if(((new_time - last_ideal_times[v]) > timeout_length) or (not cmd_msg.velocity[v] == target_velocity[v])):
                ideal_position[v] = angles[cmd_msg.names[v]]
                last_ideal_times[v] = new_time
        target_velocity = cmd_msg.velocity
        last_velocity_time = new_time

def main():
    global limb
    global target_position
    global target_velocity
    global new_position_command

    rospy.init_node("panda_translator")
    limb = ArmInterface()
    rospy.Subscriber("/panda_joint_commands", JointCommand, command_processor)
    
    rhz = rospy.Rate(1000)
    joints = limb.joint_names()
    last_loop_time = rospy.Time.now().to_sec()
    while(not rospy.is_shutdown()):
        this_loop_time = rospy.Time.now().to_sec()
        if(new_position_command):
            joint_command = dict(zip(joints, target_position))
            limb.set_joint_positions(joint_command)
            new_position_command = False
        else:
            if(not (this_loop_time - last_velocity_time > timeout_length)):
                delta = this_loop_time - last_loop_time
                new_positions = limb.joint_angles()
                for j in range(len(joints)):
                    if(abs(target_velocity[j]) <= 0.001):
                        ideal_position[j] = new_positions[joints[j]]
                    else:
                        ideal_position[j] += target_velocity[j] * delta
                #limb.set_joint_positions(new_positions)
                limb.set_joint_positions_velocities(ideal_position, target_velocity)
        last_loop_time = this_loop_time
        rhz.sleep()

if __name__ == '__main__':
    main()