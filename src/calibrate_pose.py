#!/usr/bin/env python

import message_filters
import numpy as np
import rospy
import threading

from geometry_msgs.msg import PoseStamped

cnt = 0
jk_t_cf_n3 = np.zeros((2000, 3))
cplt_event = threading.Event()

def pose_callback(cf_msg: PoseStamped, jk_msg: PoseStamped):
    global cnt

    if cnt < jk_t_cf_n3.shape[0]:
        jk_t_cf_n3[cnt, 0] = cf_msg.pose.position.x - jk_msg.pose.position.x
        jk_t_cf_n3[cnt, 1] = cf_msg.pose.position.y - jk_msg.pose.position.y
        jk_t_cf_n3[cnt, 2] = cf_msg.pose.position.z - jk_msg.pose.position.z
        cnt += 1
    else:
        cplt_event.set()

if __name__ == "__main__":
    rospy.init_node("calibrate_pose")

    cf_name = rospy.get_param("/crazy_params/crazyflie_name")
    jk_name = rospy.get_param("/crazy_params/jackal_name")

    cf_sub = message_filters.Subscriber(f"/vrpn_client_node/{cf_name}/pose", PoseStamped)
    jk_sub = message_filters.Subscriber(f"/vrpn_client_node/{jk_name}/pose", PoseStamped)
    sync = message_filters.ApproximateTimeSynchronizer([cf_sub, jk_sub], 10, 0.1)
    sync.registerCallback(pose_callback)

    cplt_event.wait()
    rospy.loginfo(f"Average jk_t_cf: {jk_t_cf_n3.mean(axis=0)}")
