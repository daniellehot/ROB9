#!/usr/bin/env python
import rospy
from nav_msgs.msg import Path
import geometry_msgs
from geometry_msgs.msg import PoseStamped, Pose, Quaternion, Vector3
from std_msgs.msg import Float32MultiArray
from nav_msgs.msg import Path
import tf2_ros
import tf2_geometry_msgs
import tf_conversions
from tf.transformations import quaternion_matrix, quaternion_from_matrix, unit_vector, quaternion_conjugate, quaternion_multiply

import numpy as np

from rob9.srv import tf2TransformPoseStampedSrv, tf2TransformPoseStampedSrvResponse
from rob9.srv import tf2TransformPathSrv, tf2TransformPathSrvResponse
from rob9.srv import tf2QuatToRotSrv, tf2QuatToRotSrvResponse
from rob9.srv import tf2QuaternionMultiplySrv, tf2QuaternionMultiplySrvResponse
from rob9.srv import tf2QuaternionConjugateSrv, tf2QuaternionConjugateSrvResponse
from rob9.srv import tf2UnitVectorSrv, tf2UnitVectorSrvResponse
#from rob9.srv import tf2getQuatSrv, tf2getQuatSrvResponse

def getRotationMatrix(req):

    R = quaternion_matrix([req.quaternion.x, req.quaternion.y, req.quaternion.z, req.quaternion.w])[:3,:3]
    msg = tf2QuatToRotSrvResponse()
    msg.rotation.data = R.flatten()
    return msg

def transformPathToFrame(req):

    newFrame = req.new_frame.data

    transformed_poses = []
    for pose in req.poses:
        pose.header.stamp = rospy.Time.now()
        transformed_pose_msg = geometry_msgs.msg.PoseStamped()
        tf_buffer.lookup_transform(pose.header.frame_id, newFrame, rospy.Time.now(), rospy.Duration(1))
        transformed_poses.append(tf_buffer.transform(pose, newFrame))

    path = Path()
    path.poses = transformed_poses

    return transformed_pose_msg


def transformToFrame(req):

    pose = req.pose
    newFrame = req.new_frame.data

    pose.header.stamp = rospy.Time.now()
    transformed_pose_msg = geometry_msgs.msg.PoseStamped()
    tf_buffer.lookup_transform(pose.header.frame_id, newFrame, rospy.Time.now(), rospy.Duration(1))
    transformed_pose_msg = tf_buffer.transform(pose, newFrame)
    return transformed_pose_msg

def quaternionMultiply(req):

    q1 = [req.q1.x, req.q1.y, req.q1.z, req.q1.w]
    q2 = [req.q2.x, req.q2.y, req.q2.z, req.q2.w]

    q = quaternion_multiply(q1, q2)

    msg = tf2QuaternionMultiplySrvResponse()
    msg.q.x, msg.q.y, msg.q.z, msg.q.w = q[0], q[1], q[2], q[3]

    return msg

def quaternionConjugate(req):

    q = [req.q.x, req.q.y, req.q.z, req.q.w]
    qc = quaternion_conjugate(q)

    msg = tf2QuaternionConjugateSrvResponse()
    msg.qc.x, msg.qc.y, msg.qc.z, msg.qc.w = qc[0], qc[1], qc[2], qc[3]

    return msg

def unitVector(req):

    v = req.v
    v = [req.v.x, req.v.y, req.v.z]
    v_norm = unit_vector(v)

    msg = tf2UnitVectorSrvResponse()
    msg.v_norm.x, msg.v_norm.y, msg.v_norm.z = v_norm[0], v_norm[1], v_norm[2]

    return msg

if __name__ == '__main__':

    baseServiceName = "/tf2/"

    rospy.init_node('tf2_service', anonymous=True)
    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)

    transformPoseStampedService = rospy.Service(baseServiceName + "transformPoseStamped", tf2TransformPoseStampedSrv, transformToFrame)
    transformPathService = rospy.Service(baseServiceName + "transformPath", tf2TransformPathSrv, transformToFrame)
    quatToRotService = rospy.Service(baseServiceName + "quatToRot", tf2QuatToRotSrv, getRotationMatrix)
    quatMulService = rospy.Service(baseServiceName + "quaternion_multiply", tf2QuaternionMultiplySrv, quaternionMultiply)
    quatConService = rospy.Service(baseServiceName + "quaternion_conjugate", tf2QuaternionConjugateSrv, quaternionConjugate)
    unitVecService = rospy.Service(baseServiceName + "unit_vector", tf2UnitVectorSrv, unitVector)

    rospy.spin()
