#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""Publish /pp_enable True only while a joystick button is held.

This is a simple "deadman" enable for autonomous control.
Default enable button is R1 on many gamepads, but you can override.

Params (private ~):
  ~joy_topic (str)        default: /joy
  ~enable_button (int)    default: 4
  ~topic (str)            default: /pp_enable
  ~rate_hz (float)        default: 30.0
"""

import rospy
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool


class ModeHold(object):
    def __init__(self):
        rospy.init_node("mode_hold", anonymous=False)

        self.joy_topic = rospy.get_param("~joy_topic", "/joy")
        self.enable_button = int(rospy.get_param("~enable_button", 4))
        self.topic = rospy.get_param("~topic", "/pp_enable")
        self.rate_hz = float(rospy.get_param("~rate_hz", 30.0))

        self._enabled = False
        self._last_buttons_len = 0

        self.pub = rospy.Publisher(self.topic, Bool, queue_size=10)
        rospy.Subscriber(self.joy_topic, Joy, self.cb_joy, queue_size=10)

        rospy.loginfo("mode_hold: hold button index %d to enable (%s)",
                      self.enable_button, self.topic)

    def cb_joy(self, msg):
        self._last_buttons_len = len(msg.buttons)
        if self.enable_button < len(msg.buttons):
            self._enabled = (msg.buttons[self.enable_button] == 1)
        else:
            self._enabled = False

    def spin(self):
        r = rospy.Rate(self.rate_hz)
        last_warn = rospy.Time(0)
        while not rospy.is_shutdown():
            self.pub.publish(Bool(data=self._enabled))
            # occasional warning if button index is out of range
            if self._last_buttons_len and self.enable_button >= self._last_buttons_len:
                if (rospy.Time.now() - last_warn).to_sec() > 2.0:
                    rospy.logwarn("mode_hold: enable_button=%d out of range (buttons=%d). "
                                  "Find R1 index with: rostopic echo -n 1 /joy",
                                  self.enable_button, self._last_buttons_len)
                    last_warn = rospy.Time.now()
            r.sleep()


if __name__ == "__main__":
    ModeHold().spin()
