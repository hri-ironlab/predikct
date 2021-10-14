#!/usr/bin/env python

# simple_disco.py: Move the fetch arm through a simple disco motion
import rospy
from sensor_msgs.msg import JointState

# WARNING: DO NOT RUN THIS RESET SCRIPT ON THE REAL ROBOT.
# This script simply sets all joints to the given values without consideration for collisions or velocity limits.
# ONLY use for quick resetting of joints in simulation.
if __name__ == '__main__':
    rospy.init_node("fetch_resetter")

    # Lists of joint angles in the same order as in joint_names
    reset_pose = [0.0, -0.5, 0.0, 0.7, 0.0, -0.2, 0.0]

    reset_publisher = rospy.Publisher("/arm_controller/joint_velocity/reset_positions", JointState, queue_size=1)

    rospy.sleep(2)

    reset_command = JointState()
    reset_command.position = reset_pose

    reset_publisher.publish(reset_command)
