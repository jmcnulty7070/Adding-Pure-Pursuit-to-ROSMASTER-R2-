#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool


class R2ModeHold(object):
    def __init__(self):
        rospy.init_node("r2_mode_hold", anonymous=False)

        self.button = rospy.get_param("~button", 5)  # typical "R2"
        self.topic = rospy.get_param("~topic", "/pp_enable")
        self.rate_hz = float(rospy.get_param("~rate_hz", 30.0))

        self.pub = rospy.Publisher(self.topic, Bool, queue_size=1, latch=True)
        rospy.Subscriber("/joy", Joy, self.cb, queue_size=10)

        self.enabled = False
        self.last = rospy.Time.now()

        rospy.loginfo("r2_mode_hold: hold button %d to enable (%s)", self.button, self.topic)

    def cb(self, msg):
        if self.button < len(msg.buttons):
            self.enabled = (msg.buttons[self.button] == 1)
        else:
            self.enabled = False

    def spin(self):
        r = rospy.Rate(self.rate_hz)
        while not rospy.is_shutdown():
            self.pub.publish(Bool(data=self.enabled))
            r.sleep()


if __name__ == "__main__":
    node = R2ModeHold()
    node.spin()

