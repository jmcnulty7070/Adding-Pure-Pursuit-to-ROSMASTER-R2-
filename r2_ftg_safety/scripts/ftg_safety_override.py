#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist


class FtgSafetyOverride(object):
    def __init__(self):
        rospy.init_node("ftg_safety_override", anonymous=False)

        self.safe_topic = rospy.get_param("~safe_topic", "/ftg_safe")
        self.in_topic = rospy.get_param("~in_topic", "/cmd_vel_auto")
        self.out_topic = rospy.get_param("~out_topic", "/cmd_vel")

        self.is_safe = True

        self.pub = rospy.Publisher(self.out_topic, Twist, queue_size=10)
        rospy.Subscriber(self.safe_topic, Bool, self.cb_safe, queue_size=10)
        rospy.Subscriber(self.in_topic, Twist, self.cb_cmd, queue_size=10)

        rospy.loginfo("ftg_safety_override: %s -> %s (safe: %s)", self.in_topic, self.out_topic, self.safe_topic)

    def cb_safe(self, msg):
        self.is_safe = bool(msg.data)

    def cb_cmd(self, msg):
        if self.is_safe:
            self.pub.publish(msg)
        else:
            self.pub.publish(Twist())


if __name__ == "__main__":
    FtgSafetyOverride()
    rospy.spin()

