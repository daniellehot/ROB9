#!/usr/bin/env python
import sys
import copy
import rospy
import moveit_commander
from moveit_commander.conversions import pose_to_list
import moveit_msgs.msg
import geometry_msgs.msg
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Pose
import std_msgs.msg
from std_msgs.msg import Int8
from math import pi
import tf2_ros
import tf2_geometry_msgs
import tf_conversions
import tf2_ros

# from robotiq_3f_gripper_articulated_msgs.msg import Robotiq3FGripperRobotOutput

def transform_frame(msg):
    transformed_pose_msg = geometry_msgs.msg.PoseStamped()
    trans = tf_buffer.lookup_transform(msg.header.frame_id, 'world', rospy.Time.now(), rospy.Duration(1.0))
    transformed_pose_msg = tf_buffer.transform(msg, "world")
    #print(transformed_pose_msg)
    send_goal_ee_pos(transformed_pose_msg)

    """
    try:
        transformed_pose_msg = tf_buffer.transform(msg, "world")
        print(transformed_pose_msg)
        send_goal_ee_pos(transformed_pose_msg)
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
        pass
    """


def move_to_ready():
    ready_pose_msg = geometry_msgs.msg.Pose()
    ready_pose_msg.position.x = 0.4071
    ready_pose_msg.position.y = 0.1361
    ready_pose_msg.position.z = 1.6743
    ready_pose_msg.orientation.x = -0.0575
    ready_pose_msg.orientation.y = 0.4495
    ready_pose_msg.orientation.z = 0.7546
    ready_pose_msg.orientation.w = 0.4745

    waypoint=[ready_pose_msg]
    (plan, fraction) = move_group.compute_cartesian_path(waypoint, 0.01, 0.0)
    move_group.execute(plan, wait=True)
    move_group.stop()
    move_group.clear_pose_targets()


def send_trajectory_to_rviz(plan):
    display_trajectory = moveit_msgs.msg.DisplayTrajectory()
    display_trajectory.trajectory_start = move_group.get_current_state()
    display_trajectory.trajectory.append(plan)
    display_trajectory_publisher.publish(display_trajectory)


def send_goal_ee_pos(msg):
    print("Sending the goal pose")
    print(msg)
    #move_group.set_pose_target(msg)
    #plan = move_group.go(wait=True)

    waypoint=[msg.pose]
    (plan, fraction) = move_group.compute_cartesian_path(waypoint, 0.01, 1.0)
    print("Fraction")
    print(fraction)
    send_trajectory_to_rviz(plan)
    #rospy.sleep(10.)

    move_group.execute(plan, wait=True)
    move_group.stop()
    move_group.clear_pose_targets()
    rospy.sleep(10.)

    #move_to_ready()
    #gripper_pub.publish(close_gripper_msg)
    print("Done")


def callback(msg):
    print("Callback")
    print(msg)
    msg.header.frame_id = "ptu_camera_color_optical_frame_real"
    transform_frame(msg)

def reset_callback(msg):
    print("Reset callback")
    if msg.data == 1:
        global reset_gripper_msg, activate_gripper_msg, pinch_gripper_msg
        move_group.set_named_target("ready")
        plan = move_group.go(wait=True)
        move_group.stop()
        #gripper_pub.publish(reset_gripper_msg)
        #gripper_pub.publish(activate_gripper_msg)
        #gripper_pub.publish(pinch_gripper_msg)
    else:
        print("Invalid input")


if __name__ == '__main__':
    rospy.init_node('moveit_subscriber', anonymous=True)
    rospy.Subscriber('pose_to_reach', PoseStamped, callback)
    rospy.Subscriber('reset_robot', Int8, reset_callback)
    gripper_pub = rospy.Publisher('gripper_controller', Int8, queue_size=1)

    moveit_commander.roscpp_initialize(sys.argv)
    move_group = moveit_commander.MoveGroupCommander("manipulator")
    move_group.set_max_acceleration_scaling_factor(0.1)
    display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                   moveit_msgs.msg.DisplayTrajectory,
                                                   queue_size=20)

    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)

    reset_gripper_msg = std_msgs.msg.Int8()
    reset_gripper_msg.data = 0
    activate_gripper_msg = std_msgs.msg.Int8()
    activate_gripper_msg.data = 1
    close_gripper_msg = std_msgs.msg.Int8()
    close_gripper_msg = 2
    open_gripper_msg = std_msgs.msg.Int8()
    open_gripper_msg.data = 3
    basic_gripper_msg = std_msgs.msg.Int8()
    basic_gripper_msg.data = 4
    pinch_gripper_msg = std_msgs.msg.Int8()
    pinch_gripper_msg.data = 5

    gripper_pub.publish(reset_gripper_msg)
    gripper_pub.publish(activate_gripper_msg)
    #gripper_pub.publish(pinch_gripper_msg)

    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
