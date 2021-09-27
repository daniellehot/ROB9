#!/usr/bin/env python
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
# from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped
from moveit_commander.conversions import pose_to_list

def send_goal_ee_pos(msg, move_group):
    """
    current_pose = move_group.get_current_pose()
    print(current_pose)
    print('======================================')
    print(msg)
    pose_goal = geometry_msgs.msg.Pose()
    pose_goal.orientation = current_pose.pose.orientation
    pose_goal.position.x = current_pose.pose.position.x
    pose_goal.position.y = current_pose.pose.position.y
    pose_goal.position.z = current_pose.pose.position.z + 0.2
    """
    pose_goal = geometry_msgs.msg.PoseStamped()
    pose_goal.header.frame_id = "world"
    pose_goal.header.stamp = rospy.Time.now()
    pose_goal.pose.orientation= msg.pose.orientation
    pose_goal.pose.position = msg.pose.position
    print(pose_goal)
    print("Sending the goal pose")
    move_group.set_pose_target(pose_goal)
    plan = move_group.go(wait=True)
    move_group.stop()
    move_group.clear_pose_targets()
    print("Done")


def send_goal_joint_pos():
    # We can get the joint values from the group and adjust some of the values:
    joint_goal = move_group.get_current_joint_values()
    joint_goal[0] = 0
    joint_goal[1] = -pi/4
    joint_goal[2] = 0
    joint_goal[3] = -pi/2
    joint_goal[4] = 0
    joint_goal[5] = pi/3
    joint_goal[6] = 0

    # The go command can be called with joint values, poses, or without any
    # parameters if you have already set the pose or joint target for the group
    print("Sending the goal pose")
    move_group.go(joint_goal, wait=True)

    # Calling ``stop()`` ensures that there is no residual movement
    move_group.stop()
    print("Done")


def send_goal_pos():
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('move_group_python_interface_tutorial', anonymous=True)
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()

    group_name = "manipulator"
    move_group = moveit_commander.MoveGroupCommander(group_name)

    display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                   moveit_msgs.msg.DisplayTrajectory,
                                                   queue_size=20)
    send_goal_ee_pos(move_group)

def callback(msg):
    moveit_commander.roscpp_initialize(sys.argv)
    move_group = moveit_commander.MoveGroupCommander("manipulator")
    send_goal_ee_pos(msg,move_group)


def subscriber():
    rospy.init_node('moveit_subscriber', anonymous=True)
    rospy.Subscriber('pose_to_reach', PoseStamped, callback)
    rospy.spin()

if __name__ == '__main__':
    try:
        subscriber()
    except rospy.ROSInterruptException:
        pass
