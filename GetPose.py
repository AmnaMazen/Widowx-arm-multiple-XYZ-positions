#!/usr/bin/env python

import sys
import rospy
import moveit_commander
from tf.transformations import euler_from_quaternion

# Initialize moveit_commander and a ROS node
moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('get_current_xyz', anonymous=True)

# Initialize the robot
robot = moveit_commander.RobotCommander()

# Initialize the MoveGroupCommander for your robot's arm
group_name = "widowx_arm"  # Replace with your group name
group = moveit_commander.MoveGroupCommander(group_name)

# Get the current pose of the end effector
current_pose = group.get_current_pose()
current_position = current_pose.pose.position
current_orientation = current_pose.pose.orientation

# Convert quaternion to roll, pitch, yaw
quaternion = (
    current_orientation.x,
    current_orientation.y,
    current_orientation.z,
    current_orientation.w
)
roll, pitch, yaw = euler_from_quaternion(quaternion)

# Print out the XYZ coordinates and orientation angles
print("Current Position:")
print("X: {:.2f}".format(current_position.x))
print("Y: {:.2f}".format(current_position.y))
print("Z: {:.2f}".format(current_position.z))

print("\nCurrent Orientation (Roll, Pitch, Yaw):")
print("Roll: {:.2f}".format(roll))
print("Pitch: {:.2f}".format(pitch))
print("Yaw: {:.2f}".format(yaw))

