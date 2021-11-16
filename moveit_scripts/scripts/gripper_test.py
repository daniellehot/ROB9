#!/usr/bin/env python
import sys
import rospy
from std_msgs.msg import *


def callback(msg):
    if msg.data == 2:
        gripper_pub.publish(close_gripper_msg)
    if msg.data == 3:
        gripper_pub.publish(open_gripper_msg)
    print("Done")


if __name__ == '__main__':
    rospy.init_node('gripper_test')
    rospy.Subscriber('gripper_test_sub', Int8, callback)
    gripper_pub = rospy.Publisher('gripper_controller', Int8, queue_size=10)
    rospy.sleep(0.1)

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
    gripper_pub.publish(open_gripper_msg)
    gripper_pub.publish(pinch_gripper_msg)

    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
