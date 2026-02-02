#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""Joystick E-Stop / Teleop Lock for twist_mux.

Publishes twist_mux_msgs/Lock to a lock topic (default: /teleop_lock).
Two modes:
  - hold_mode: lock is active only while button is held
  - toggle_mode: each press toggles lock (debounced)

When locked, we also publish a zero Twist to /cmd_vel_teleop as an extra safety layer.

Params (private ~):
  ~joy_topic (str)           default: /joy
  ~lock_topic (str)          default: /teleop_lock
  ~cmd_zero_topic (str)      default: /cmd_vel_teleop
  ~estop_button (int)        default: 3
  ~hold_mode (bool)          default: true
  ~toggle_mode (bool)        default: false
  ~debounce_s (float)        default: 0.25
  ~rate_hz (float)           default: 20
"""

import rospy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from twist_mux_msgs.msg import Lock


class JoyEStopLock(object):
    def __init__(self):
        rospy.init_node("joy_estop_lock", anonymous=False)

        self.joy_topic = rospy.get_param("~joy_topic", "/joy")
        self.lock_topic = rospy.get_param("~lock_topic", "/teleop_lock")
        self.cmd_zero_topic = rospy.get_param("~cmd_zero_topic", "/cmd_vel_teleop")

        self.estop_button = int(rospy.get_param("~estop_button", 3))
        self.hold_mode = bool(rospy.get_param("~hold_mode", True))
        self.toggle_mode = bool(rospy.get_param("~toggle_mode", False))
        self.debounce_s = float(rospy.get_param("~debounce_s", 0.25))
        self.rate_hz = float(rospy.get_param("~rate_hz", 20.0))

        self.locked = False
        self._last_press_time = rospy.Time(0)
        self._last_pressed = 0

        self.lock_pub = rospy.Publisher(self.lock_topic, Lock, queue_size=10)
        self.zero_pub = rospy.Publisher(self.cmd_zero_topic, Twist, queue_size=10)

        rospy.Subscriber(self.joy_topic, Joy, self.cb_joy, queue_size=10)

        rospy.loginfo("joy_estop_lock: button=%d lock_topic=%s (hold=%s toggle=%s)",
                      self.estop_button, self.lock_topic, self.hold_mode, self.toggle_mode)

        # publish loop for lock + optional zeroing
        self.spin()

    def cb_joy(self, msg):
        pressed = 0
        if self.estop_button < len(msg.buttons):
            pressed = 1 if msg.buttons[self.estop_button] == 1 else 0

        now = rospy.Time.now()

        if self.toggle_mode:
            # rising edge
            if pressed == 1 and self._last_pressed == 0:
                if (now - self._last_press_time).to_sec() >= self.debounce_s:
                    self.locked = not self.locked
                    self._last_press_time = now
                    rospy.logwarn("E-STOP TOGGLE -> %s", "LOCKED" if self.locked else "UNLOCKED")
        elif self.hold_mode:
            # locked while held
            self.locked = (pressed == 1)

        self._last_pressed = pressed

    def _publish_lock(self):
        msg = Lock()
        msg.stamp = rospy.Time.now()
        msg.locked = bool(self.locked)
        self.lock_pub.publish(msg)

    def _publish_zero_if_locked(self):
        if self.locked:
            self.zero_pub.publish(Twist())

    def spin(self):
        r = rospy.Rate(self.rate_hz)
        while not rospy.is_shutdown():
            self._publish_lock()
            self._publish_zero_if_locked()
            r.sleep()


if __name__ == "__main__":
    JoyEStopLock()
