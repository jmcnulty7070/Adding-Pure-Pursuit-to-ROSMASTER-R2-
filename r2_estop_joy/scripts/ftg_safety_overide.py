#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""Simple LiDAR safety override for autonomous cmd_vel.

Input:  /cmd_vel_auto
Output: /cmd_vel_safety

Behavior:
  - If enable topic is False -> output zero
  - If min front distance < stop_dist -> output zero
  - If between stop_dist and slow_dist -> scale linear speed down (min_scale)
  - Angular speed optionally limited by max_steer_rad_s

Params (~):
  scan_topic (str)        default: /scan
  auto_cmd_topic (str)    default: /cmd_vel_auto
  out_cmd_topic (str)     default: /cmd_vel_safety
  enable_topic (str)      default: /pp_enable

  front_fov_deg (float)   default: 60
  stop_dist_m (float)     default: 0.55
  slow_dist_m (float)     default: 1.2
  min_scale (float)       default: 0.2
  max_steer_rad_s (float) default: 1.5
  rate_hz (float)         default: 30
"""

import math
import rospy
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan


class FtgSafetyOverride(object):
    def __init__(self):
        rospy.init_node("ftg_safety_override", anonymous=False)

        self.scan_topic = rospy.get_param("~scan_topic", "/scan")
        self.auto_cmd_topic = rospy.get_param("~auto_cmd_topic", "/cmd_vel_auto")
        self.out_cmd_topic = rospy.get_param("~out_cmd_topic", "/cmd_vel_safety")
        self.enable_topic = rospy.get_param("~enable_topic", "/pp_enable")

        self.front_fov_deg = float(rospy.get_param("~front_fov_deg", 60.0))
        self.stop_dist_m = float(rospy.get_param("~stop_dist_m", 0.55))
        self.slow_dist_m = float(rospy.get_param("~slow_dist_m", 1.2))
        self.min_scale = float(rospy.get_param("~min_scale", 0.2))
        self.max_steer_rad_s = float(rospy.get_param("~max_steer_rad_s", 1.5))
        self.rate_hz = float(rospy.get_param("~rate_hz", 30.0))

        self.enabled = False
        self.last_cmd = Twist()
        self.min_front = None

        self.pub = rospy.Publisher(self.out_cmd_topic, Twist, queue_size=10)

        rospy.Subscriber(self.enable_topic, Bool, self.cb_enable, queue_size=10)
        rospy.Subscriber(self.auto_cmd_topic, Twist, self.cb_cmd, queue_size=10)
        rospy.Subscriber(self.scan_topic, LaserScan, self.cb_scan, queue_size=10)

        rospy.loginfo("ftg_safety_override: %s -> %s (enable: %s, scan: %s)",
                      self.auto_cmd_topic, self.out_cmd_topic, self.enable_topic, self.scan_topic)

        self.spin()

    def cb_enable(self, msg):
        self.enabled = bool(msg.data)

    def cb_cmd(self, msg):
        self.last_cmd = msg

    def cb_scan(self, msg):
        # compute minimum range in +/- front_fov/2 around 0 rad
        fov = math.radians(self.front_fov_deg)
        # indices corresponding to [-fov/2, +fov/2]
        a_min = msg.angle_min
        a_inc = msg.angle_increment
        if a_inc == 0.0:
            self.min_front = None
            return

        def idx(angle):
            return int((angle - a_min) / a_inc)

        i0 = max(0, idx(-fov/2.0))
        i1 = min(len(msg.ranges)-1, idx(fov/2.0))
        m = None
        for r in msg.ranges[i0:i1+1]:
            if math.isfinite(r) and r > 0.01:
                m = r if m is None else min(m, r)
        self.min_front = m

    def compute_safe_cmd(self):
        if not self.enabled:
            return Twist()

        if self.min_front is None:
            # No scan yet -> be safe
            return Twist()

        if self.min_front <= self.stop_dist_m:
            return Twist()

        # scale in (stop, slow)
        scale = 1.0
        if self.min_front < self.slow_dist_m:
            # linear interpolation from min_scale at stop_dist to 1.0 at slow_dist
            t = (self.min_front - self.stop_dist_m) / max(1e-6, (self.slow_dist_m - self.stop_dist_m))
            scale = self.min_scale + t * (1.0 - self.min_scale)

        out = Twist()
        out.linear.x = self.last_cmd.linear.x * scale
        out.linear.y = 0.0
        out.angular.z = max(-self.max_steer_rad_s, min(self.max_steer_rad_s, self.last_cmd.angular.z))
        return out

    def spin(self):
        r = rospy.Rate(self.rate_hz)
        while not rospy.is_shutdown():
            self.pub.publish(self.compute_safe_cmd())
            r.sleep()


if __name__ == "__main__":
    FtgSafetyOverride()
