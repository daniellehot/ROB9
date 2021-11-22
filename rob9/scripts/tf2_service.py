#!/usr/bin/env python
import rospy
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Pose, Quaternion
from std_msgs.msg import Float32MultiArray
import tf2_ros
import tf2_geometry_msgs
import tf_conversions
from tf.transformations import quaternion_matrix, quaternion_from_matrix

import numpy as np

from rob9.srv import tf2TransformPoseStampedSrv, tf2TransformPoseStampedSrvResponse
from rob9.srv import tf2QuatToRotSrv, tf2QuatToRotSrvResponse
#from rob9.srv import tf2getQuatSrv, tf2getQuatSrvResponse

def getRotationMatrix(req):

    R = quaternion_matrix([req.quaternion.x, req.quaternion.y, req.quaternion.z, req.quaternion.w])[:3,:3]
    msg = tf2QuatToRotSrvResponse()
    msg.rotation.data = R.flatten()
    return msg

def transformToFrame(req):

    pose = req.pose
    newFrame = req.new_frame.data

    pose.header.stamp = rospy.Time.now()
    transformed_pose_msg = geometry_msgs.msg.PoseStamped()
    tf_buffer.lookup_transform(pose.header.frame_id, newFrame, rospy.Time.now(), rospy.Duration(1))
    transformed_pose_msg = tf_buffer.transform(pose, newFrame)
    return transformed_pose_msg

if __name__ == '__main__':

    baseServiceName = "/tf2/"

    rospy.init_node('tf2_service', anonymous=True)
    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)

    transformPoseStampedService = rospy.Service(baseServiceName + "transformPoseStamped", tf2TransformPoseStampedSrv, transformToFrame)
    quatToRotService = rospy.Service(baseServiceName + "quatToRot", tf2QuatToRotSrv, getRotationMatrix)

    rospy.spin()
