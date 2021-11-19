#!/usr/bin/env python3
import rospy
import numpy as np
from geometry_msgs.msg import Pose, PoseStamped
from std_msgs.msg import String, Float32, Int32, Header
import tf2_ros
import tf2_geometry_msgs
import tf_conversions
from tf.transformations import quaternion_matrix, quaternion_from_matrix
from rob9.msg import *
import rob9Utils.transformations as transform

class Position(object):
    """docstring for Position."""

    def __init__(self, x = 0, y = 0, z = 0):
        self.x = x
        self.y = y
        self.z = z

    def __add__(self, val):
        return (self.x + val[0], self.y + val[1], self.z + val[2])

    def getVector(self, format="row"):
        if format == "row":
            return np.array([self.x, self.y, self.z])
        elif format == "column":
            return np.array([[self.x], [self.y], [self.z]])
        else:
            print("Invalid vector format, chose row or column ")
            return 0

class Orientation(object):
    """Orientation represented as a quaternion"""

    def __init__(self, qw = 1, qx = 0, qy = 0, qz = 0):
        self.w = qw
        self.x = qx
        self.y = qy
        self.z = qz

    def __repr__(self):
        return "Orientation (wxyz)" + str(self.w) + " " + str(self.x) + " " + str(self.y) + " " + str(self.z) + "\n"

    def getRotationMatrix(self):
        return quaternion_matrix([self.x, self.y, self.z, self.w])[:3,:3]

    def getVector(self, format = "wxyz"):
        if format == "wxyz":
            return np.array([self.w, self.x, self.y. self.z])
        elif format == "xyzw":
            return np.array([self.x, self.y, self.z, self.w])
        else:
            print("Invalid quaternion format, chose wxyz or xyzw ")
            return 0

    def fromRotationMatrix(self, R):
        self.__init__()
        if R.shape[0] < 4 or R.shape[1] < 4:
            for i in range(4 - R.shape[0]):
                R = np.c_[R, np.zeros((R.shape[1], 1))]
            for i in range(4 - R.shape[0]):
                R = np.r_[R, np.zeros((1, R.shape[1]))]
        R[-1, -1] = 1

        q = quaternion_from_matrix(R)

        self.x = q[0]
        self.y = q[1]
        self.z = q[2]
        self.w = q[3]

        return self

class Grasp(object):
    """docstring for Grasp."""

    def __init__(self, frame_id = "ptu_camera_color_optical_frame"):
        self.position = Position()
        self.orientation = Orientation()
        self.score = 0.0
        self.frame_id = frame_id
        self.tool_id = None
        self.affordance_id = None
        self.object_instance = None

    def __repr__(self):

        rep = ["---------------- \n" + "Grasp\n" + " Position \n" + "  x: " + str(round(self.position.x, 3)) + "\n  y: "
            + str(round(self.position.y, 3)) + "\n  z: " + str(round(self.position.z, 3))
            + "\n Orientation: \n  qx: " + str(round(self.orientation.x, 3))
            + "\n  qy: " + str(round(self.orientation.y, 3))
            + "\n  qz: " + str(round(self.orientation.z, 3))
            + "\n  qw: " + str(round(self.orientation.w, 3))
            + "\n Header \n  Frame: " + str(self.frame_id)
            + "\n  Score: " + str(self.score)
            + "\n  tool: " + str(self.tool_id)
            + "\n  affordance: " + str(self.affordance_id)
            + "\n  object instance: " + str(self.object_instance)
            ]

        return rep[0]

    def fromPoseStampedMsg(self, msg):

        self.__init__()

        self.frame_id = msg.header.frame_id
        self.position = Position(x = msg.pose.position.x,
                                 y = msg.pose.position.y,
                                 z = msg.pose.position.z)

        self.orientation = Orientation(qx = msg.pose.orientation.x,
                                        qy = msg.pose.orientation.y,
                                        qz = msg.pose.orientation.z,
                                        qw = msg.pose.orientation.w,)

        return self

    def fromGraspMsg(self, msg):

        self.__init__()

        self.frame_id = msg.frame_id.data
        self.score = msg.score.data
        self.tool_id = msg.tool_id.data
        self.affordance_id = msg.affordance_id.data
        self.object_instance = msg.object_instance.data

        self.position = Position(x = msg.pose.position.x,
                                 y = msg.pose.position.y,
                                 z = msg.pose.position.z)

        self.orientation = Orientation(qx = msg.pose.orientation.x,
                                        qy = msg.pose.orientation.y,
                                        qz = msg.pose.orientation.z,
                                        qw = msg.pose.orientation.w,)

        return self

    def getRotationMatrix(self):
        return self.orientation.getRotationMatrix()

    def setObjectInstance(self, instance):
        self.object_instance = int(instance)

    def transformToFrame(self, tf_buffer, frame_dest):
        current_pose = self.toPoseStampedMsg()
        new_f = transform.transformToFrame(tf_buffer, current_pose, current_pose.header.frame_id, frame_dest)

        self.position = Position(x = new_f.pose.position.x, y = new_f.pose.position.y, z = new_f.pose.position.z)
        self.orientation = Orientation(qx = new_f.pose.orientation.x, qy = new_f.pose.orientation.y, qz = new_f.pose.orientation.z, qw = new_f.pose.orientation.w)
        self.frame_id = new_f.header.frame_id

    def toPoseStampedMsg(self):

        header = Header()
        header.stamp = rospy.Time.now()
        header.frame_id = str(self.frame_id)

        pose = Pose()
        pose.position.x = self.position.x
        pose.position.y = self.position.y
        pose.position.z = self.position.z
        pose.orientation.x = self.orientation.x
        pose.orientation.y = self.orientation.y
        pose.orientation.z = self.orientation.z
        pose.orientation.w = self.orientation.w

        msg = PoseStamped()
        msg.header = header
        msg.pose = pose

        return msg

    def toGraspMsg(self):
        """ returns a grasp message """
        mFrame_id = String()
        mFrame_id.data = str(self.frame_id)

        mScore = Float32()
        mScore.data = float(self.score)

        mTool_id = Int32()
        mTool_id.data = int(self.tool_id) if self.tool_id is not None else 0

        mAffordance_id = Int32()
        mAffordance_id.data = int(self.affordance_id) if self.affordance_id is not None else 0

        mObject_instance = Int32()
        mObject_instance.data = int(self.object_instance) if self.object_instance is not None else -1

        header = Header()
        header.stamp = rospy.Time.now()
        header.frame_id = str(self.frame_id)

        pose = Pose()
        pose.position.x = self.position.x
        pose.position.y = self.position.y
        pose.position.z = self.position.z
        pose.orientation.x = self.orientation.x
        pose.orientation.y = self.orientation.y
        pose.orientation.z = self.orientation.z
        pose.orientation.w = self.orientation.w

        msg = GraspMsg()
        msg.frame_id = mFrame_id
        msg.score = mScore
        msg.tool_id = mTool_id
        msg.affordance_id = mAffordance_id
        msg.object_instance = mObject_instance
        msg.header = header
        msg.pose = pose

        return msg

if __name__ == '__main__':
    rospy.init_node('rob9_grasp_test', anonymous=True)
    g = Grasp()
    msg = g.toGraspMsg()
    print(msg)

    msg = g.toPoseStampedMsg()
    print(msg)

    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)
    rospy.sleep(2)
    g.transformToFrame(tf_buffer, frame_dest="world")
    print(g)
