#!/usr/bin/env python

import rospy

from geometry_msgs.msg import PoseStamped

if __name__ == "__main__":
    rospy.init_node("land_test")
    pub = rospy.Publisher("/crazy_land/crazyflie/ctrl", PoseStamped, queue_size=10)

    rospy.sleep(1)
    rospy.loginfo("Starting to test land...")
    msg = PoseStamped()
    msg.header.frame_id = "LAND"
    msg.header.stamp = rospy.Time.now() + rospy.Duration(3)
    msg.pose.position.z = 0.0
    pub.publish(msg)
