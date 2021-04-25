#!/usr/bin/env python

import math
import threading
import time

import cflib.crtp
import rospy

from cflib.crazyflie import Crazyflie
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Joy
from std_msgs.msg import String

class FlightControl:

    URI = "radio://0/80/2M/E7E7E7E7E7"

    def __init__(self):
        cflib.crtp.init_drivers(enable_debug_driver=False)
        self.cf = Crazyflie(rw_cache="./cache")
        cf_name = rospy.get_param("/crazy_params/crazyflie_name")

        self._btn_override = rospy.get_param("/crazy_params/btn_cross")
        self._btn_land = rospy.get_param("/crazy_params/btn_triangle")
        self._link_is_valid = False
        self._initialized = False
        self._is_flying = False
        self._joy_override = False
        self._connect_event = None
        self._disconnect_event = None
        self._cmd_lock = threading.Lock()

        self._add_callbacks()
        self._initiate_connection()
        self._init_sequence()

        self._meas_event = None
        self.vicon_sub = rospy.Subscriber(f"/vrpn_client_node/{cf_name}/pose",
                                          PoseStamped, self._send_ext_pose)
        self.joy_sub = rospy.Subscriber("/bluetooth_teleop/joy",
                                        Joy, self._joy_control)
        self.cmd_sub = rospy.Subscriber("/crazy_land/crazyflie/ctrl",
                                        PoseStamped, self._send_command)
        self.status_pub = rospy.Publisher("/crazy_land/crazyflie/status",
                                          String, queue_size=10)

        self._wait_for_measurements()
        self._initialized = True
        rospy.loginfo("Crazyflie initialization complete")
        self.status_pub.publish(String("Initialized"))

        rospy.on_shutdown(self._shutdown)

    def _add_callbacks(self):
        self.cf.connected.add_callback(self._connected)
        self.cf.connection_failed.add_callback(self._connection_failed)
        self.cf.disconnected.add_callback(self._disconnected)

    def _wait_for_measurements(self, num_measurements: int = 100):
        rospy.loginfo("Waiting for enough measurements...")
        self._meas_event = threading.Event()
        for i in range(num_measurements):
            self._meas_event.wait()
            self._meas_event.clear()
        self._meas_event = None

    def _initiate_connection(self):
        rospy.loginfo("Connecting with crazyflie...")
        self._connect_event = threading.Event()
        self.cf.open_link(self.URI)
        self._connect_event.wait()
        self._connect_event = None

    def _init_sequence(self):
        rospy.loginfo("Running crazyflie init sequence...")
        # enable EKF
        self.cf.param.set_value("stabilizer.estimator", "2")

        # EKF orientation uncertainty (loser then default)
        self.cf.param.set_value("locSrv.extQuatStdDev", 0.05)

        # enable high level commander
        self.cf.param.set_value("commander.enHighLevel", "1")

        # reset Kalman Estimator
        self.cf.param.set_value("kalman.resetEstimation", "1")
        time.sleep(0.1)
        self.cf.param.set_value("kalman.resetEstimation", "0")

    def _send_command(self, msg: PoseStamped):
        self._cmd_lock.acquire()
        if not self._initialized or self._joy_override:
            self._cmd_lock.release()
            return

        duration = max((msg.header.stamp - rospy.Time.now()).to_sec(), 2.0)
        if msg.header.frame_id == "TAKEOFF" and not self._is_flying:
            rospy.loginfo("Crazyflie taking off...")
            self.cf.high_level_commander.takeoff(msg.pose.position.z, duration)
            self._is_flying = True
        elif msg.header.frame_id == "LAND" and self._is_flying:
            rospy.loginfo("Crazyflie landing...")
            self.cf.high_level_commander.land(msg.pose.position.z, duration)
            self._is_flying = False
        elif msg.header.frame_id == "SHUTDOWN" and self._is_flying:
            rospy.loginfo("Crazyflie shutting down...")
            self.cf.commander.send_stop_setpoint()
            self._is_flying = False
        elif msg.header.frame_id == "FLYTO":
            angle = 2 * math.atan2(msg.pose.orientation.z, msg.pose.orientation.w)
            self.cf.commander.send_position_setpoint(msg.pose.position.x,
                                                     msg.pose.position.y,
                                                     msg.pose.position.z,
                                                     math.degrees(angle))
            self._is_flying = True
            rospy.logdebug(f"Crazyflie flying towards {msg.pose.position} @ {angle}")
        elif msg.header.frame_id == "RETURN" and self._is_flying:
            rospy.loginfo(f"Crazyflie returning towards {msg.pose.position}")
            fly_duration = 5.0
            sleep = 0.1

            # fly to position
            self.cf.high_level_commander.go_to(msg.pose.position.x,
                                               msg.pose.position.y,
                                               msg.pose.position.z,
                                               0.0, 5.)
            time.sleep(5.0)

            # land
            self.cf.high_level_commander.land(0.0, 5.0)
            time.sleep(5.0)

            self._is_flying = False

        self._cmd_lock.release()

    def _joy_control(self, msg: Joy):
        if not self._initialized:
            return

        self._cmd_lock.acquire()
        if msg.buttons[self._btn_override] and not self._joy_override and self._is_flying:
            rospy.loginfo("Crazyflie joystick override triggered")
            self._joy_override = True
            # return home on joystick override
            self.cf.high_level_commander.go_to(0.0, 0.0, 1.0, 0.0, 5.)

        if not self._joy_override:
            self._cmd_lock.release()
            return

        if msg.buttons[self._btn_land] and self._is_flying:
            rospy.loginfo("Crazyflie joystick landing inplace...")
            self.cf.high_level_commander.land(0.0, 3.0)
            self._is_flying = False
        self._cmd_lock.release()

    def _send_ext_pose(self, msg: PoseStamped):
        if self._link_is_valid:
            position = msg.pose.position
            orientation = msg.pose.orientation
            self.cf.extpos.send_extpose(position.x, position.y, position.z,
                                        orientation.x, orientation.y, orientation.z, orientation.w)
            if self._meas_event:
                self._meas_event.set()

    def _shutdown(self):
        if self._link_is_valid:
            self._disconnect_event = threading.Event()
            self.cf.close_link()
            self._disconnect_event.wait()
            self._disconnect_event = None

    def _connected(self, link_uri):
        rospy.loginfo(f"Connected with crazyflie @ {link_uri}")
        if self._connect_event:
            self._connect_event.set()
        self._link_is_valid = True

    def _connection_failed(self, link_uri, msg):
        rospy.logerr(f"Connection with crazyflie failed @ {link_uri} : {msg}")
        rospy.signal_shutdown("Connection with crazflie falied")
        if self._connect_event:
            self._connect_event.set()

    def _disconnected(self, link_uri):
        rospy.loginfo(f"Disconnected with crazyflie @ {link_uri}")
        self._link_is_valid = False
        if self._disconnect_event:
            self._disconnect_event.set()


if __name__ == "__main__":
    rospy.init_node("flight_control")

    fc = FlightControl()

    rospy.spin()

