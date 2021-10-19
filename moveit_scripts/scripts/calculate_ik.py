#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped
import moveit_commander
from moveit_msgs.srv import *
from moveit_msgs.msg import PositionIKRequest, RobotState, MoveItErrorCodes

def get_ik(msg, robot_state):
    rospy.wait_for_service('compute_ik')
    request_msg = moveit_msgs.msg.PositionIKRequest()
    request_msg.group_name = "manipulator"
    request_msg.robot_state = robot_state
    request_msg.avoid_collisions = False
    request_msg.pose_stamped = msg
    request_msg.timeout = rospy.Duration(10.0)
    request_msg.attempts = 2
    #print "request_msg \n", request_msg
    try:
        calculate_ik = rospy.ServiceProxy("compute_ik", GetPositionIK)
        robot_state_out = calculate_ik(request_msg)
        print "robot_state_out \n", robot_state_out
        #add_two_ints = rospy.ServiceProxy('add_two_ints', AddTwoInts)
        #resp1 = add_two_ints(x, y)
        #return resp1.sum
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)


if __name__ == "__main__":
    rospy.init_node('calculate_ik', anonymous=True)
    moveit_commander.roscpp_initialize(sys.argv)
    robot = moveit_commander.RobotCommander()
    if len(sys.argv) == 8:
        x = float(sys.argv[1])
        y = float(sys.argv[2])
        z = float(sys.argv[3])
        qx = float(sys.argv[4])
        qy = float(sys.argv[5])
        qz = float(sys.argv[6])
        qw = float(sys.argv[7])
    else:
        print("requires 7 inputs")
        sys.exit(1)
    print("Requesting tke IK for the pose")
    pose_msg = geometry_msgs.msg.PoseStamped()
    pose_msg.pose.position.x = x
    pose_msg.pose.position.y = y
    pose_msg.pose.position.z = z
    pose_msg.pose.orientation.x = qx
    pose_msg.pose.orientation.y = qy
    pose_msg.pose.orientation.z = qz
    pose_msg.pose.orientation.w = qw
    pose_msg.header.frame_id = "world"
    pose_msg.header.stamp = rospy.Time.now()
    robot_state = moveit_msgs.msg.RobotState()
    robot_state = robot.get_current_state()
    get_ik(pose_msg, robot_state)
