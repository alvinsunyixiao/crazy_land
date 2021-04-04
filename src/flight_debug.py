#!/usr/bin/env python

import math
import time

import cflib.crtp
import rospy

from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie.syncLogger import SyncLogger
from geometry_msgs.msg import Pose, PoseStamped

class PoseSender:
    def __init__(self):
        crazyflie_name = rospy.get_param("crazyflie_name", default="alvin_cf")
        rospy.init_node("flight_debug", anonymous=True)
        self.vicon_sub = rospy.Subscriber(f"/vrpn_client_node/{crazyflie_name}/pose",
                                          PoseStamped, self._on_packet)
        self.on_pose = None

    def _on_packet(self, msg: PoseStamped):
        if self.on_pose:
            self.on_pose(msg.pose)


class FlightDebuger:

    URI = "radio://0/80/2M/E7E7E7E7E7"

    def __init__(self):
        cflib.crtp.init_drivers(enable_debug_driver=False)
        self.cf = Crazyflie(rw_cache="./cache")

    def run(self, pose_sender: PoseSender):
        # wait for vicon vrpn node boot up
        time.sleep(5)
        with SyncCrazyflie(self.URI, cf=self.cf) as scf:
            # register pose sender
            pose_sender.on_pose = self.update_pose

            # activate kalman estimator
            scf.cf.param.set_value("stabilizer.estimator", "2")
            scf.cf.param.set_value("locSrv.extQuatStdDev", 0.03)

            # actiavte high level commander
            scf.cf.param.set_value("commander.enHighLevel", "1")

            # actiave mellinger controller
            # scf.cf.param.set_value("stabilizer.controller", "2")

            # reset Kalman filter
            self.reset_estimator(scf)

            self.debug_blocking_log(scf)

    def update_pose(self, msg: Pose):
        self.cf.extpos.send_extpose(msg.position.x, msg.position.y, msg.position.z,
                                    msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w)

    def reset_estimator(self, scf: SyncCrazyflie):
        scf.cf.param.set_value("kalman.resetEstimation", "1")
        time.sleep(0.1)
        scf.cf.param.set_value("kalman.resetEstimation", "0")

        # wait for Kalman filter to converge
        time.sleep(5)

    def debug_blocking_log(self, scf: SyncCrazyflie):
        log_config = LogConfig(name="Kalman Filter Stats", period_in_ms=400)
        #log_config.add_variable("kalman.stateX", "float")
        #log_config.add_variable("kalman.stateY", "float")
        #log_config.add_variable("kalman.stateZ", "float")
        log_config.add_variable("kalman.q0", "float")
        log_config.add_variable("kalman.q1", "float")
        log_config.add_variable("kalman.q2", "float")
        log_config.add_variable("kalman.q3", "float")

        with SyncLogger(scf, log_config) as logger:
            for log_entry in logger:
                print(log_entry)


if __name__ == "__main__":
    pose_sender = PoseSender()
    debuger = FlightDebuger()
    debuger.run(pose_sender)

    rospy.spin()
