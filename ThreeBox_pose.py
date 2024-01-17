#!/usr/bin/env python

import sys
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import PoseStamped, Pose

# Initialize moveit_commander and rospy
moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('move_group_python_interface_tutorial', anonymous=True)

# Initialize the robot and scene interface
robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()

# Initialize the MoveGroupCommander for your robot
group_name = "widowx_arm"
group = moveit_commander.MoveGroupCommander(group_name)

# Set the reference frame for pose targets
reference_frame = "base_link"
group.set_pose_reference_frame(reference_frame)

# Set the end-effector link
eef_link = group.get_end_effector_link()

group.allow_replanning(True)


#Set the tolerance of positioon and orientation
group.set_goal_position_tolerance(0.02)
group.set_goal_orientation_tolerance(0.02)

#Set the max acceleration and velocity
group.set_max_acceleration_scaling_factor(0.5)
group.set_max_velocity_scaling_factor(0.5)


############# Position 1: pick up the first box #####################################

#x, y, z coordinates in meters
target_position = [0.16, 0.22, 0.08]

 #orientation in quaternion (roll, pitch, yaw)
target_orientation = quaternion_from_euler(-2.79, 1.56, -1.84)
#Set the pose target
pose_goal = geometry_msgs.msg.Pose() 
pose_goal.orientation.x = target_orientation[0]
pose_goal.orientation.y = target_orientation[1]
pose_goal.orientation.z = target_orientation[2]
pose_goal.orientation.w = target_orientation[3]
pose_goal.position.x = target_position[0]
pose_goal.position.y = target_position[1]
pose_goal.position.z = target_position[2]
group.set_pose_target(pose_goal, eef_link)

traj = group.plan()

group.execute(traj)
rospy.sleep(1)

# Plan and execute the motion
group.go(wait=True)

group.stop()



############# Position 2: Open gripper #####################################

rospy.sleep(1);
group_name2 = "widowx_gripper"
group2 = moveit_commander.MoveGroupCommander(group_name2)
joint_goal2 = group2.get_current_joint_values()
joint_goal2[0] = 0.02

# The go command can be called with joint values, poses, or without any
# parameters if you have already set the pose or joint target for the group
group2.go(joint_goal2, wait=True)

# Calling ``stop()`` ensures that there is no residual movement
group2.stop()
group2.clear_pose_targets()


############# Position 3: Close gripper #####################################

rospy.sleep(1);
group_name2 = "widowx_gripper"
group2 = moveit_commander.MoveGroupCommander(group_name2)
joint_goal2 = group2.get_current_joint_values()
joint_goal2[0] = 0.007

# The go command can be called with joint values, poses, or without any
# parameters if you have already set the pose or joint target for the group
group2.go(joint_goal2, wait=True)

# Calling ``stop()`` ensures that there is no residual movement
group2.stop()
group2.clear_pose_targets()



############# Position 4: Put the first box #####################################

#x, y, z coordinates in meters
target_position = [0.07, -0.24, 0.07]

 #orientation in quaternion (roll, pitch, yaw)
target_orientation = quaternion_from_euler(-0.51, 1.54, -1.78)  # Example orientation in quaternion (roll, pitch, yaw)


#Set the pose target
pose_goal = geometry_msgs.msg.Pose() 
pose_goal.orientation.x = target_orientation[0]
pose_goal.orientation.y = target_orientation[1]
pose_goal.orientation.z = target_orientation[2]
pose_goal.orientation.w = target_orientation[3]
pose_goal.position.x = target_position[0]
pose_goal.position.y = target_position[1]
pose_goal.position.z = target_position[2]
group.set_pose_target(pose_goal, eef_link)

traj = group.plan()

group.execute(traj)
rospy.sleep(1)

# Plan and execute the motion
group.go(wait=True)

group.stop()


############# Position 5: Open gripper #####################################

rospy.sleep(1);
group_name2 = "widowx_gripper"
group2 = moveit_commander.MoveGroupCommander(group_name2)
joint_goal2 = group2.get_current_joint_values()
joint_goal2[0] = 0.02

# The go command can be called with joint values, poses, or without any
# parameters if you have already set the pose or joint target for the group
group2.go(joint_goal2, wait=True)

# Calling ``stop()`` ensures that there is no residual movement
group2.stop()
group2.clear_pose_targets()



############# Position 6: Pick up the second box #####################################

#x, y, z coordinates in meters
target_position = [0.26, -0.00, 0.10]

 #orientation in quaternion (roll, pitch, yaw)
target_orientation = quaternion_from_euler(-2.94, 1.52, -2.95)


#Set the pose target
pose_goal = geometry_msgs.msg.Pose() 
pose_goal.orientation.x = target_orientation[0]
pose_goal.orientation.y = target_orientation[1]
pose_goal.orientation.z = target_orientation[2]
pose_goal.orientation.w = target_orientation[3]
pose_goal.position.x = target_position[0]
pose_goal.position.y = target_position[1]
pose_goal.position.z = target_position[2]
group.set_pose_target(pose_goal, eef_link)

traj = group.plan()

group.execute(traj)
rospy.sleep(1)

# Plan and execute the motion
group.go(wait=True)

group.stop()


############# Position 7: Close gripper #####################################

rospy.sleep(1);
group_name2 = "widowx_gripper"
group2 = moveit_commander.MoveGroupCommander(group_name2)
joint_goal2 = group2.get_current_joint_values()
joint_goal2[0] = 0.007
#joint_goal2[1] = 0.03

# The go command can be called with joint values, poses, or without any
# parameters if you have already set the pose or joint target for the group
group2.go(joint_goal2, wait=True)

# Calling ``stop()`` ensures that there is no residual movement
group2.stop()
group2.clear_pose_targets()



############# Position 8: Put the Second box #####################################

#x, y, z coordinates in meters
target_position = [0.07, -0.24, 0.07]

 #orientation in quaternion (roll, pitch, yaw)
target_orientation = quaternion_from_euler(-0.51, 1.54, -1.78)  

#Set the pose target
pose_goal = geometry_msgs.msg.Pose() 
pose_goal.orientation.x = target_orientation[0]
pose_goal.orientation.y = target_orientation[1]
pose_goal.orientation.z = target_orientation[2]
pose_goal.orientation.w = target_orientation[3]
pose_goal.position.x = target_position[0]
pose_goal.position.y = target_position[1]
pose_goal.position.z = target_position[2]
group.set_pose_target(pose_goal, eef_link)

traj = group.plan()

group.execute(traj)
rospy.sleep(1)

# Plan and execute the motion
group.go(wait=True)

group.stop()


############# Position 9: Open gripper #####################################

rospy.sleep(1);
group_name2 = "widowx_gripper"
group2 = moveit_commander.MoveGroupCommander(group_name2)
joint_goal2 = group2.get_current_joint_values()
joint_goal2[0] = 0.02

# The go command can be called with joint values, poses, or without any
# parameters if you have already set the pose or joint target for the group
group2.go(joint_goal2, wait=True)

# Calling ``stop()`` ensures that there is no residual movement
group2.stop()
group2.clear_pose_targets()



############# Position 10: Pick up the Third box #####################################

#x, y, z coordinates in meters
target_position = [0.28, -0.19, 0.10]

 #orientation in quaternion (roll, pitch, yaw)
target_orientation = quaternion_from_euler(3.14, 1.55, 2.54)


#Set the pose target
pose_goal = geometry_msgs.msg.Pose() 
pose_goal.orientation.x = target_orientation[0]
pose_goal.orientation.y = target_orientation[1]
pose_goal.orientation.z = target_orientation[2]
pose_goal.orientation.w = target_orientation[3]
pose_goal.position.x = target_position[0]
pose_goal.position.y = target_position[1]
pose_goal.position.z = target_position[2]
group.set_pose_target(pose_goal, eef_link)

traj = group.plan()

group.execute(traj)
rospy.sleep(1)

# Plan and execute the motion
group.go(wait=True)

group.stop()


############# Position 11: Close gripper #####################################

rospy.sleep(1);
group_name2 = "widowx_gripper"
group2 = moveit_commander.MoveGroupCommander(group_name2)
joint_goal2 = group2.get_current_joint_values()
joint_goal2[0] = 0.007
#joint_goal2[1] = 0.03

# The go command can be called with joint values, poses, or without any
# parameters if you have already set the pose or joint target for the group
group2.go(joint_goal2, wait=True)

# Calling ``stop()`` ensures that there is no residual movement
group2.stop()
group2.clear_pose_targets()


############# Position 12: Put the Third box #####################################

#x, y, z coordinates in meters
target_position = [0.07, -0.24, 0.07]

 #orientation in quaternion (roll, pitch, yaw)
target_orientation = quaternion_from_euler(-0.51, 1.54, -1.78)  

#Set the pose target
pose_goal = geometry_msgs.msg.Pose() 
pose_goal.orientation.x = target_orientation[0]
pose_goal.orientation.y = target_orientation[1]
pose_goal.orientation.z = target_orientation[2]
pose_goal.orientation.w = target_orientation[3]
pose_goal.position.x = target_position[0]
pose_goal.position.y = target_position[1]
pose_goal.position.z = target_position[2]
group.set_pose_target(pose_goal, eef_link)

traj = group.plan()

group.execute(traj)
rospy.sleep(1)

# Plan and execute the motion
group.go(wait=True)

group.stop()


############# Position 13: Open gripper #####################################

rospy.sleep(1);
group_name2 = "widowx_gripper"
group2 = moveit_commander.MoveGroupCommander(group_name2)
joint_goal2 = group2.get_current_joint_values()
joint_goal2[0] = 0.02

# The go command can be called with joint values, poses, or without any
# parameters if you have already set the pose or joint target for the group
group2.go(joint_goal2, wait=True)

# Calling ``stop()`` ensures that there is no residual movement
group2.stop()
group2.clear_pose_targets()


############# Position 14: Go to the default pose #####################################

#x, y, z coordinates in meters
target_position = [0.27, -0.00, 0.26]

 #orientation in quaternion (roll, pitch, yaw)
target_orientation = quaternion_from_euler(3.14, 1.52, 3.14)  

#Set the pose target
pose_goal = geometry_msgs.msg.Pose() 
pose_goal.orientation.x = target_orientation[0]
pose_goal.orientation.y = target_orientation[1]
pose_goal.orientation.z = target_orientation[2]
pose_goal.orientation.w = target_orientation[3]
pose_goal.position.x = target_position[0]
pose_goal.position.y = target_position[1]
pose_goal.position.z = target_position[2]
group.set_pose_target(pose_goal, eef_link)

traj = group.plan()

group.execute(traj)
rospy.sleep(1)

# Plan and execute the motion
group.go(wait=True)

group.stop()