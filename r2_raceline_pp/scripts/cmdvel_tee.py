#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import Twist


class CmdVelTee(object):
    def __init__(self):
        in_topic = rospy.get_param("~in_topic", "/cmd_vel")
        out_topic = rospy.get_param("~out_topic", "/cmd_vel_teleop")

        self.pub = rospy.Publisher(out_topic, Twist, queue_size=10)
        self.sub = rospy.Subscriber(in_topic, Twist, self.cb, queue_size=10)
        rospy.loginfo("cmdvel_tee: %s -> %s", in_topic, out_topic)

    def cb(self, msg):
        self.pub.publish(msg)


if __name__ == "__main__":
    rospy.init_node("cmdvel_tee")
    CmdVelTee()
    rospy.spin()

