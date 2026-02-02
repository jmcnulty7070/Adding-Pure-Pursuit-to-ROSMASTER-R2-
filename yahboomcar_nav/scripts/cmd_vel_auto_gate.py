#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy

class CmdVelAutoGate(object):
    def __init__(self):
        self.joy_topic = rospy.get_param("~joy_topic", "/joy")
        self.in_auto_topic = rospy.get_param("~in_auto_topic", "/cmd_vel_auto")
        self.out_auto_topic = rospy.get_param("~out_auto_topic", "/cmd_vel_auto_gated")

        self.deadman_button_index = int(rospy.get_param("~deadman_button_index", 0))
        self.require_deadman = bool(rospy.get_param("~require_deadman", True))

        self.rate_hz = float(rospy.get_param("~rate", 20.0))
        self.joy_timeout = float(rospy.get_param("~joy_timeout", 0.75))
        self.auto_timeout = float(rospy.get_param("~auto_timeout", 0.5))

        # IMPORTANT FIX:
        # When blocked, publish ONE zero to stop, then go silent so twist_mux times out
        self.publish_stop_once_on_block = bool(rospy.get_param("~publish_stop_once_on_block", True))

        self.last_joy_time = rospy.Time(0)
        self.last_auto_time = rospy.Time(0)

        self.deadman_pressed = False
        self.last_output_allowed = False

        self.latest_auto = Twist()

        self.pub = rospy.Publisher(self.out_auto_topic, Twist, queue_size=1)

        rospy.Subscriber(self.joy_topic, Joy, self.cb_joy, queue_size=1)
        rospy.Subscriber(self.in_auto_topic, Twist, self.cb_auto, queue_size=1)

        rospy.loginfo("cmd_vel_auto_gate: deadman_button_index=%d require_deadman=%s in=%s out=%s joy=%s",
                      self.deadman_button_index, str(self.require_deadman),
                      self.in_auto_topic, self.out_auto_topic, self.joy_topic)

    def cb_joy(self, msg):
        self.last_joy_time = rospy.Time.now()

        pressed = False
        if self.deadman_button_index >= 0 and self.deadman_button_index < len(msg.buttons):
            pressed = (msg.buttons[self.deadman_button_index] == 1)

        self.deadman_pressed = pressed

    def cb_auto(self, msg):
        self.last_auto_time = rospy.Time.now()
        self.latest_auto = msg

    def allowed(self):
        now = rospy.Time.now()
        joy_ok = (now - self.last_joy_time).to_sec() <= self.joy_timeout
        if not joy_ok and self.require_deadman:
            return False
        if not self.require_deadman:
            return True
        return self.deadman_pressed

    @staticmethod
    def zero_twist():
        return Twist()

    def spin(self):
        rate = rospy.Rate(self.rate_hz)

        while not rospy.is_shutdown():
            now = rospy.Time.now()
            allowed = self.allowed()

            auto_fresh = (now - self.last_auto_time).to_sec() <= self.auto_timeout

            if allowed and auto_fresh:
                # Pass autonomy through
                self.pub.publish(self.latest_auto)
                self.last_output_allowed = True
            else:
                # Blocked:
                # publish a single "stop" when transitioning from allowed->blocked,
                # then STOP publishing so twist_mux falls back to teleop after its timeout.
                if self.publish_stop_once_on_block and self.last_output_allowed:
                    self.pub.publish(self.zero_twist())
                self.last_output_allowed = False

            rate.sleep()

if __name__ == "__main__":
    rospy.init_node("cmd_vel_auto_gate")
    node = CmdVelAutoGate()
    node.spin()

