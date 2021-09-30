#!/usr/bin/env python
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
from moveit_commander.conversions import pose_to_list
import geometry_msgs.msg
from geometry_msgs.msg import PoseStamped
import std_msgs.msg
from std_msgs.msg import Int8
from math import pi
import tf2_ros
import tf2_geometry_msgs
from robotiq_3f_gripper_articulated_msgs.msg import Robotiq3FGripperRobotOutput

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

def genCommand(msg, command):
    """Update the command according to the character entered by the user."""

    if msg.data == 'a':
        command = Robotiq3FGripperRobotOutput();
        command.rACT = 1
        command.rGTO = 1
        command.rSPA = 255
        command.rFRA = 150

    if msg.data == 'r':
        command = Robotiq3FGripperRobotOutput();
        command.rACT = 0

    if msg.data == 'c':
        command.rPRA = 255

    if msg.data == 'o':
        command.rPRA = 0

    if msg.data == 'b':
        command.rMOD = 0

    if msg.data == 'p':
        command.rMOD = 1

    if msg.data == 'w':
        command.rMOD = 2

    if msg.data == 's':
        command.rMOD = 3

    return command



def callback(msg):
    print("Callback", msg)
    transform_frame(msg)

def gripper_callback(msg):
    command = Robotiq3FGripperRobotOutput();
    # activate
    if msg.data == 0:
        command.rACT = 1
        command.rGTO = 1
        command.rSPA = 255
        command.rFRA = 150
    # reset
    if msg.data == -1:
        command.rACT = 0
    # close
    if msg.data == 1:
        command.rPRA = 255
    # open
    if msg.data == 2:
        command.rPRA = 0
    # basic
    if msg.data == 4:
        command.rMOD = 0
    # pinch
    if msg.data == 5:
        command.rMOD = 1
    # wide
    if msg.data == 6:
        command.rMOD = 2
    # scissor
    if msg.data == 7:
        command.rMOD = 3
    pub_command.publish(command)



if __name__ == '__main__':
    rospy.init_node('moveit_subscriber', anonymous=True)
    rospy.Subscriber('pose_to_reach', PoseStamped, callback)
    rospy.Subscriber('gripper_control', Int8, gripper_callback)
    pub_command = rospy.Publisher('Robotiq3FGripperRobotOutput', Robotiq3FGripperRobotOutput)
    pub_int = rospy.Publisher('gripper_control', std_msgs.msg.Int8, queue_size=1)

    moveit_commander.roscpp_initialize(sys.argv)
    move_group = moveit_commander.MoveGroupCommander("manipulator")
    display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                   moveit_msgs.msg.DisplayTrajectory,
                                                   queue_size=20)
    #pub_int.publish(initialise_gripper_msg)

    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)

    command = Robotiq3FGripperRobotOutput();

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
