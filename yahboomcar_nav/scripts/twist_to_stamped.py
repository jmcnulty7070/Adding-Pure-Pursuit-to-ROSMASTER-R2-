#!/usr/bin/env python
# coding: utf-8

import rospy
from geometry_msgs.msg import Twist, TwistStamped

class TwistToStamped(object):
    def __init__(self):
        in_topic  = rospy.get_param("~in_topic",  "/cmd_vel_teleop")
        out_topic = rospy.get_param("~out_topic", "/cmd_vel_teleop_stamped")
        self.frame_id = rospy.get_param("~frame_id", "base_link")

        self.pub = rospy.Publisher(out_topic, TwistStamped, queue_size=10)
        self.sub = rospy.Subscriber(in_topic, Twist, self.cb, queue_size=10)

        rospy.loginfo("twist_to_stamped: %s (Twist) -> %s (TwistStamped)", in_topic, out_topic)

    def cb(self, msg):
        out = TwistStamped()
        out.header.stamp = rospy.Time.now()
        out.header.frame_id = self.frame_id
        out.twist = msg
        self.pub.publish(out)

if __name__ == "__main__":
    rospy.init_node("twist_to_stamped", anonymous=False)
    TwistToStamped()
    rospy.spin()

