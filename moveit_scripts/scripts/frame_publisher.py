#!/usr/bin/env python
import sys
import copy
import rospy
import moveit_commander
from moveit_commander.conversions import pose_to_list
import moveit_msgs.msg
import geometry_msgs
from geometry_msgs.msg import Pose, PoseStamped
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Int8
from math import pi
import tf2_ros
import tf2_geometry_msgs
import tf_conversions


def transform_frame(msg, desired_frame):
    transformed_pose_msg = geometry_msgs.msg.PoseStamped()
    trans = tf_buffer.lookup_transform(msg.header.frame_id, desired_frame, rospy.Time.now(), rospy.Duration(1.0))
    transformed_pose_msg = tf_buffer.transform(msg, desired_frame)
    return transformed_pose_msg

    """
    try:
        transformed_pose_msg = tf_buffer.transform(msg, "world")
        print(transformed_pose_msg)
        send_goal_ee_pos(transformed_pose_msg)
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
        pass
    """


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
    transformed_msg = geometry_msgs.msg.PoseStamped()
    transformed_msg = transform_frame(msg, "world")
    print("Publishing goal frame")
    t = geometry_msgs.msg.TransformStamped()
    t.header.stamp = rospy.Time.now()
    t.header.frame_id = transformed_msg.header.frame_id
    t.child_frame_id = "goal_to_reach"
    t.transform.translation.x = transformed_msg.pose.position.x
    t.transform.translation.y = transformed_msg.pose.position.y
    t.transform.translation.z = transformed_msg.pose.position.z
    t.transform.rotation.x = transformed_msg.pose.orientation.x
    t.transform.rotation.y = transformed_msg.pose.orientation.y
    t.transform.rotation.z = transformed_msg.pose.orientation.z
    t.transform.rotation.w = transformed_msg.pose.orientation.w
    br_goal.sendTransform(t)

def publish_waypoint_frame(msg):
    transformed_msg = geometry_msgs.msg.PoseStamped()
    transformed_msg = transform_frame(msg, "world")
    print("Publishing waypoint frame")
    t = geometry_msgs.msg.TransformStamped()
    t.header.stamp = rospy.Time.now()
    t.header.frame_id = transformed_msg.header.frame_id
    t.child_frame_id = "waypoint_to_reach"
    t.transform.translation.x = transformed_msg.pose.position.x
    t.transform.translation.y = transformed_msg.pose.position.y
    t.transform.translation.z = transformed_msg.pose.position.z
    t.transform.rotation.x = transformed_msg.pose.orientation.x
    t.transform.rotation.y = transformed_msg.pose.orientation.y
    t.transform.rotation.z = transformed_msg.pose.orientation.z
    t.transform.rotation.w = transformed_msg.pose.orientation.w
    print(t)
    br_waypoint.sendTransform(t)


def callback(msg):
    # if demo == True and msg.header.frame_id == "ptu_camera_color_optical_frame" :
        # msg.header.frame_id = "ptu_camera_color_optical_frame_real"
    publish_grasp_frame(msg)
    publish_goal_frame(msg)


def waypoint_callback(msg):
    print("waypoint callback")
    print(msg)
    # if demo == True and msg.header.frame_id == "ptu_camera_color_optical_frame" :
        # msg.header.frame_id = "ptu_camera_color_optical_frame_real"
    # print(msg)
    publish_waypoint_frame(msg)


def pc_callback(msg):
    msg.header.frame_id = "ptu_camera_color_optical_frame_real"
    pc_pub.publish(msg)


if __name__ == '__main__':
    demo = False
    if len(sys.argv) > 1:
        if sys.argv[1] == 'demo':
            demo = True
        else:
            print("Invalid input argument")

    rospy.init_node('frame_publisher', anonymous=True)
    rospy.Subscriber('pose_to_reach', PoseStamped, callback)
    rospy.Subscriber('pose_to_reach_waypoint', PoseStamped, waypoint_callback)

    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)
    br_goal= tf2_ros.StaticTransformBroadcaster()
    br_grasp = tf2_ros.StaticTransformBroadcaster()
    br_waypoint = tf2_ros.StaticTransformBroadcaster()

    if demo == True:
        rospy.Subscriber('sensors/realsense/pointcloudGeometry/static', PointCloud2, pc_callback)
        pc_pub = rospy.Publisher('sensors/realsense/pointcloudGeometry/static_corrected', PointCloud2, queue_size=1)

    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
