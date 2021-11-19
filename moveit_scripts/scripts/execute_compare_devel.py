#!/usr/bin/env python
import sys
import copy
import rospy

from moveit_scripts.srv import *
from moveit_scripts.msg import *
from grasp_aff_association.srv import *

import moveit_commander
import moveit_msgs
#from moveit_commander.conversions import pose_to_list
#from moveit_msgs.srv import *
#import moveit_msgs.msg
#from moveit_msgs.msg import PositionIKRequest, RobotState, MoveItErrorCodes
import geometry_msgs.msg
from geometry_msgs.msg import Pose, PoseStamped, PoseArray
from nav_msgs.msg import Path
import std_msgs.msg
from std_msgs.msg import Int8
from math import pi
import tf2_ros
import tf2_geometry_msgs
import tf_conversions
import tf2_ros
# from robotiq_3f_gripper_articulated_msgs.msg import Robotiq3FGripperRobotOutput


def move_to_ready():
    print("Moving to ready")
    move_group.set_named_target("ready")
    plan = move_group.go(wait=True)
    move_group.stop()
    move_group.clear_pose_targets()
    gripper_pub.publish(open_gripper_msg)
    print("Done")

def move_to_handover():
    print("Moving to pre-handover pose")
    move_group.set_named_target("ready")
    plan = move_group.go(wait=True)
    move_group.stop()
    move_group.clear_pose_targets()

    print("Moving to handover pose")
    move_group.set_named_target("handover")
    plan = move_group.go(wait=True)
    move_group.stop()
    move_group.clear_pose_targets()
    #gripper_pub.publish(open_gripper_msg)
    print("Done")

def compare_current_and_start_state(robot_trajectory_msg):
    same_flag = True
    joint_values_current = move_group.get_current_joint_values()
    joint_values_start = robot_trajectory_msg.joint_trajectory.points[0].positions

    if not isinstance(joint_values_start, list):
        print("good call")
        joint_values_start = list(joint_values_start)

    for i in range(6):
        joint_values_start[i] = round(joint_values_start[i], 2)
        joint_values_current[i] = round(joint_values_current[i], 2)

    if (joint_values_current == joint_values_start):
        print("Start state equals current")
    else:
        print("Start state does not equal the current state")
        print("Current_joint_values " + str(joint_values_current))
        print("--------------------------------------")
        print("Trajectory " + str(robot_trajectory_msg.joint_trajectory.points[0].positions))
        print("Moving to the start state")
        joint_goal = joint_values_start
        #joint_goal[0] = joint_values_start[0]
        #joint_goal[1] = joint_values_start[1]
        #joint_goal[2] = joint_values_start[2]
        #joint_goal[3] = joint_values_start[3]
        #joint_goal[4] = joint_values_start[4]
        #joint_goal[5] = joint_values_start[5]

        raw_input("Press Enter when you are ready to move the robot")
        move_group.go(joint_goal, wait=True)
        move_group.stop()
        move_group.clear_pose_targets()




def send_trajectory_to_rviz(plan):
    print("Trajectory was sent to RViZ")
    display_trajectory = moveit_msgs.msg.DisplayTrajectory()
    display_trajectory.trajectory_start = robot.get_current_state()
    display_trajectory.trajectory.append(plan)
    display_trajectory_publisher.publish(display_trajectory)


def rosbag_callback(path_msg):
    print("rosbag_callback")
    resp_trajectories = get_trajectories(path_msg)
    id = tool_id
    id = str(id)

    id_list_duplicates = []
    for i in range(len(resp_trajectories.trajectories.trajectories)):
        id_list_duplicates.append(resp_trajectories.trajectories.trajectories[i].joint_trajectory.header.frame_id)
    id_list = list(dict.fromkeys(id_list_duplicates))
    print("id_list_duplicates " + str(id_list_duplicates))
    print("id_list " + str(id_list))

    plans = []
    goal_poses = []
    for i in range(len(resp_trajectories.trajectories.trajectories)):
        if resp_trajectories.trajectories.trajectories[i].joint_trajectory.header.frame_id == id:
            plans.append(resp_trajectories.trajectories.trajectories[i])

    for i in range(len(resp_trajectories.trajectories_poses.poses)):
        if resp_trajectories.trajectories_poses.poses[i].header.frame_id == id:
            goal_poses.append(resp_trajectories.trajectories_poses.poses[i])

    #print("I have sampled these trajectories "  + str(len(plans)))
    #print("I have sampled these goal poses "  + str(len(goal_poses)))

    waypoint_msg = geometry_msgs.msg.PoseStamped()
    waypoint_msg.header.frame_id = "world"
    waypoint_msg.header.stamp = rospy.Time.now()
    waypoint_msg.pose = goal_poses[0].pose
    pub_waypoint.publish(waypoint_msg)

    goal_msg = geometry_msgs.msg.PoseStamped()
    goal_msg.header.frame_id = "world"
    goal_msg.header.stamp = rospy.Time.now()
    goal_msg.pose = goal_poses[1].pose
    pub_grasp.publish(goal_msg)

    for i in range(2):
        compare_current_and_start_state(plans[i])
        send_trajectory_to_rviz(plans[i])
        raw_input("Press Enter when you are ready to move the robot")
        move_group.execute(plans[i], wait=True)
        move_group.stop()
        move_group.clear_pose_targets()

    #receivedGripperCommand = False
    #rate = rospy.Rate(10)
    #subGrupper = rospy.Subscriber("gripper_controller", Int8, callbackGripper)
    #gripper_pub.publish(close_gripper_msg)

    #while receivedGripperCommand == False:
    #    gripper_pub.publish(close_gripper_msg)
    #    rate.sleep()
    #rospy.sleep(4.)
    raw_input("Press Enter when you are ready to move the robot to the handover pose")
    move_to_handover()
    raw_input("Press Enter when you are ready to move the robot back to the ready pose")
    move_to_ready()


if __name__ == '__main__':
    tool_id = -1
    if len(sys.argv) > 1:
        tool_id = int(sys.argv[1])
        if isinstance(tool_id, int):
            print("tool_id = " + str(tool_id))
        else:
            print("Invalid input argument")
            exit()

    print("Init execute_compare_devel")
    rospy.init_node('execute_compare_devel', anonymous=True)
    rospy.Subscriber('Daniel', Path, rosbag_callback)
    pub_grasp = rospy.Publisher('pose_to_reach', PoseStamped, queue_size=10)
    pub_waypoint = rospy.Publisher('pose_to_reach_waypoint', PoseStamped, queue_size=10)
    display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=20)
    rospy.sleep(0.1)

    moveit_commander.roscpp_initialize(sys.argv)
    robot = moveit_commander.RobotCommander()
    move_group = moveit_commander.MoveGroupCommander("manipulator")

    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)

    print("Services init")
    rospy.wait_for_service('get_trajectories')
    get_trajectories = rospy.ServiceProxy('get_trajectories', GetTrajectories)
    print("Waiting in callback")

    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
