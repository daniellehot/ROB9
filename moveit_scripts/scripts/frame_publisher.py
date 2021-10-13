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


def publish_grasp_frame(msg):
    print("Publishing grasp frame")
    t = geometry_msgs.msg.TransformStamped()
    t.header.stamp = rospy.Time.now()
    t.header.frame_id = msg.header.frame_id
    t.child_frame_id = "grasp_to_reach"
    t.transform.translation.x = msg.pose.position.x
    t.transform.translation.y = msg.pose.position.y
    t.transform.translation.z = msg.pose.position.z
    t.transform.rotation.x = msg.pose.orientation.x
    t.transform.rotation.y = msg.pose.orientation.y
    t.transform.rotation.z = msg.pose.orientation.z
    t.transform.rotation.w = msg.pose.orientation.w
    br_grasp.sendTransform(t)


def publish_goal_frame(msg):
    print("Publishing goal frame")
    t = geometry_msgs.msg.TransformStamped()
    t.header.stamp = rospy.Time.now()
    t.header.frame_id = msg.header.frame_id
    t.child_frame_id = "goal_to_reach"
    t.transform.translation.x = msg.pose.position.x
    t.transform.translation.y = msg.pose.position.y
    t.transform.translation.z = msg.pose.position.z
    t.transform.rotation.x = msg.pose.orientation.x
    t.transform.rotation.y = msg.pose.orientation.y
    t.transform.rotation.z = msg.pose.orientation.z
    t.transform.rotation.w = msg.pose.orientation.w
    br_goal.sendTransform(t)


def transform_frame(msg):
    transformed_pose_msg = geometry_msgs.msg.PoseStamped()
    trans = tf_buffer.lookup_transform(msg.header.frame_id, 'world', rospy.Time.now(), rospy.Duration(1.0))
    print("Transformation")
    print(trans)
    transformed_pose_msg = tf_buffer.transform(msg, "world")
    publish_goal_frame(transformed_pose_msg)

    """
    try:
        transformed_pose_msg = tf_buffer.transform(msg, "world")
        print(transformed_pose_msg)
        send_goal_ee_pos(transformed_pose_msg)
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
        pass
    """


def callback(msg):
    if replace_frame_id == True and msg.header.frame_id == "ptu_camera_color_optical_frame" :
        msg.header.frame_id = "ptu_camera_color_optical_frame_real"
    publish_grasp_frame(msg)
    transform_frame(msg)


if __name__ == '__main__':
    replace_frame_id = False
    if len(sys.argv) > 1:
        if sys.argv[1] == 'demo':
            replace_frame_id = True
        else:
            print("Invalid input argument")

    rospy.init_node('frame_publisher', anonymous=True)
    rospy.Subscriber('pose_to_reach', PoseStamped, callback)

    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)
    br_goal= tf2_ros.StaticTransformBroadcaster()
    br_grasp = tf2_ros.StaticTransformBroadcaster()

    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
