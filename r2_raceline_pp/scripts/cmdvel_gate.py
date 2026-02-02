#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool


class CmdVelGate(object):
    def __init__(self):
        rospy.init_node("cmdvel_gate", anonymous=False)

        self.enable_topic = rospy.get_param("~enable_topic", "/pp_enable")
        self.in_topic = rospy.get_param("~in_topic", "/cmd_vel_auto_raw")
        self.out_topic = rospy.get_param("~out_topic", "/cmd_vel_auto")

        self.enabled = False

        self.pub = rospy.Publisher(self.out_topic, Twist, queue_size=10)
        rospy.Subscriber(self.enable_topic, Bool, self.cb_enable, queue_size=10)
        rospy.Subscriber(self.in_topic, Twist, self.cb_cmd, queue_size=10)

        rospy.loginfo("cmdvel_gate: %s -> %s (enable: %s)", self.in_topic, self.out_topic, self.enable_topic)

    def cb_enable(self, msg):
        self.enabled = bool(msg.data)

    def cb_cmd(self, msg):
        if self.enabled:
            self.pub.publish(msg)
        else:
            # If not enabled, publish zero cmd (safe)
            self.pub.publish(Twist())


if __name__ == "__main__":
    CmdVelGate()
    rospy.spin()

