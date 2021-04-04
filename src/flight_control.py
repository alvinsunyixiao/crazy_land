#!/usr/bin/env python

import threading

import cflib.crtp
import rospy

from cflib.crazyflie import Crazyflie
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Joy

class FlightControl:

    URI = "radio://0/80/2M/E7E7E7E7E7"

    def __init__(self):
        cflib.crtp.init_drivers(enable_debug_driver=False)
        self.cf = Crazyflie(rw_cache="./cache")
        self.vicon_sub = rospy.Subscriber("/vrpn_client_node/alvin_cf/pose",
                                          PoseStamped, self._send_ext_pose)
        self.joy_sub = rospy.Subscriber("/bluetooth_teleop/joy",
                                        Joy, self._joy_control)
        self.cmd_sub = rospy.Subscriber("crazy_land/cmd_crazy",
                                        PoseStamped, self._send_command)

        self._btn_override = rospy.get_param("/crazy_params/btn_cross")
        self._btn_override = rospy.get_param("/crazy_params/btn_triangle")
        self._link_is_valid = False
        self._is_flying = False
        self._joy_override = False
        self._num_measurements = 0
        self._joy_lock = threading.Lock()
        self._cmd_lock = threading.Lock()

        self._add_callbacks()
        self._initiate_connection()
        rospy.on_shutdown(self._shutdown)

    def _add_callbacks(self):
        self.cf.connected.add_callback(self._connected)
        self.cf.connection_failed.add_callback(self._connection_failed)
        self.cf.disconnected.add_callback(self._disconnected)

    def _initiate_connection(self):
        rospy.loginfo("Connecting with crazyflie ...")
        self._connect_event = threading.Event()
        self.cf.open_link(self.URI)
        self._connect_event.wait()
        self._connect_event = None

    def _send_command(self, msg: PoseStamped):
        if self._num_measurements < 100 or self._joy_override:
            return

        self._cmd_lock.acquire()
        duration = (msg.header.stamp - rospy.Time.now()).to_sec()
        if msg.header.frame_id == "TAKEOFF" and not self._is_flying:
            self.cf.high_level_commander.takeoff(msg.pose.position.z, duration)
            self._is_flying = True
        elif msg.header.frame_id == "LAND" and self._is_flying:
            self.cf.high_level_commander.land(msg.pose.position.z, duration)
            self._is_flying = False
        elif msg.header.frame_id == "FLYTO" and self._is_flying:
            self.cf.high_level_commander.go_to(msg.pose.position.x,
                                               msg.pose.position.y,
                                               msg.pose.position.z,
                                               0, duration)
        self._cmd_lock.release()

    def _joy_control(self, msg: Joy):
        if self._num_measurements < 100:
            return

        self._joy_lock.acquire()
        if msg.buttons[self._btn_override]:
            self._joy_override = True
        if not self._joy_override:
            self._joy_lock.release()
            return
        self._joy_lock.release()

        self._cmd_lock.acquire()
        if msg.buttons[self._btn_land] and self._is_flying:
            self.cf.high_level_commander.land(0.0, 3.0)
            self._is_flying = False
        self._cmd_lock.release()

    def _send_ext_pose(self, msg: PoseStamped):
        if self._link_is_valid:
            position = msg.pose.position
            orientation = msg.pose.orientation
            self.cf.extpos.send_extpose(position.x, position.y, position.z,
                                        orientation.x, orientation.y, orientation.z, orientation.w)
            self._num_measurements += 1

    def _shutdown(self):
        if self._link_is_valid:
            self._disconnect_event = Event()
            self.cf.close_link()
            self._disconnect_event.wait()
            self._disconnect_event = None

    def _connected(self, link_uri):
        rospy.loginfo(f"Connected with crazyflie @ {link_uri}")
        self._link_is_valid = True
        if self._connect_event:
            self._connect_event.set()

    def _connection_failed(self, link_uri, msg):
        rospy.logerr(f"Connection with crazyflie failed @ {link_uri} : {msg}")
        if self._connect_event:
            self._connect_event.set()
        rospy.signal_shutdown("Connection with crazflie falied")

    def _disconnected(self, link_uri):
        rospy.loginfo(f"Disconnected with crazyflie @ {link_uri}")
        self._link_is_valid = False
        if self._disconnect_event:
            self._disconnect_event.set()


if __name__ == "__main__":
    rospy.init_node("flight_control")

    fc = FlightControl()

    rospy.spin()


