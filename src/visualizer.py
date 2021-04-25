#!/usr/bin/env python

import serial
import struct
import message_filters
import numpy as np
import rospy

from scipy.spatial.transform import Rotation as R
from geometry_msgs.msg import PoseStamped

class Visualizer:
    def __init__(self):
        cf_name = rospy.get_param("/crazy_params/crazyflie_name")
        jk_name = rospy.get_param("/crazy_params/jackal_name")

        cf_sub = message_filters.Subscriber(f"/vrpn_client_node/{cf_name}/pose", PoseStamped)
        jk_sub = message_filters.Subscriber(f"/vrpn_client_node/{jk_name}/pose", PoseStamped)
        self.sync_sub = message_filters.ApproximateTimeSynchronizer([cf_sub, jk_sub], 10, 0.1)
        self.ser = serial.Serial(rospy.get_param("/crazy_params/serial_port"))
        self.bound_accuracy = rospy.get_param("/crazy_params/bound_accuracy")
        self.jk_t_target = np.array(rospy.get_param("/crazy_params/jk_t_cf"))[:, None]
        self.jk_R_target = R.from_quat(rospy.get_param("/crazy_params/jk_R_cf"))

    def run(self):
        self.sync_sub.registerCallback(self.pose_callback)
        rospy.spin()

    def pose_callback(self, cf_msg: PoseStamped, jk_msg: PoseStamped):
        world_R_jk = R.from_quat([
            jk_msg.pose.orientation.x,
            jk_msg.pose.orientation.y,
            jk_msg.pose.orientation.z,
            jk_msg.pose.orientation.w,
        ])

        world_t_jk = np.array([
            jk_msg.pose.position.x,
            jk_msg.pose.position.y,
            jk_msg.pose.position.z,
        ])[:, None]

        world_R_target = world_R_jk * self.jk_R_target
        world_t_target = world_R_jk.as_matrix() @ self.jk_t_target + world_t_jk

        world_t_cf = np.array([
            cf_msg.pose.position.x,
            cf_msg.pose.position.y,
            cf_msg.pose.position.z,
        ])[:, None]

        # relative position
        target_t_cf = world_R_target.inv().as_matrix() @ (world_t_cf - world_t_target)
        target_t_cf_int = np.round(target_t_cf / self.bound_accuracy).astype(int).flatten()

        self.ser.write(struct.pack("<ii", target_t_cf_int[0], target_t_cf_int[1]))

if __name__ == "__main__":
    rospy.init_node("visualizer")
    viz = Visualizer()
    viz.run()
