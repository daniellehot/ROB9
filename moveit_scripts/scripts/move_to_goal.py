#!/usr/bin/env python
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from geometry_msgs.msg import PoseStamped
from moveit_commander.conversions import pose_to_list
import tf2_ros
import tf2_geometry_msgs

def transform_frame(msg):
    transformed_pose_msg = geometry_msgs.msg.PoseStamped()
    try:
        transformed_pose_msg = tf_buffer.transform(msg, "world")
        send_goal_ee_pos(transformed_pose_msg)
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
        pass

def send_goal_ee_pos(msg):
    print("Sending the goal pose")
    print(msg)
    move_group.set_pose_target(msg)
    plan = move_group.go(wait=True)
    move_group.stop()
    move_group.clear_pose_targets()
    print("Done")


def callback(msg):
    print("Callback", msg)
    transform_frame(msg)


if __name__ == '__main__':
    rospy.init_node('moveit_subscriber', anonymous=True)
    rospy.Subscriber('pose_to_reach', PoseStamped, callback)

    moveit_commander.roscpp_initialize(sys.argv)
    move_group = moveit_commander.MoveGroupCommander("manipulator")
    display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                   moveit_msgs.msg.DisplayTrajectory,
                                                   queue_size=20)

    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)

    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

"""
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
"""
