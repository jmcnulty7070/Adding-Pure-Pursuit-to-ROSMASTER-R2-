#!/usr/bin/env python
# -*- coding: utf-8 -*-

import math
import yaml

import rospy
import tf2_ros
from geometry_msgs.msg import Twist


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


class PurePursuit(object):
    def __init__(self):
        self.path_yaml = rospy.get_param('~path_yaml')
        self.frame_id = rospy.get_param('~frame_id', 'map')
        self.base_frame = rospy.get_param('~base_frame', 'base_link')

        self.lookahead = float(rospy.get_param('~lookahead_m', 1.2))
        self.v_max = float(rospy.get_param('~v_max', 0.8))
        self.v_min = float(rospy.get_param('~v_min', 0.4))
        self.a_lat_max = float(rospy.get_param('~a_lat_max', 1.2))
        self.accel_limit = float(rospy.get_param('~accel_limit', 0.8))

        self.out_topic = rospy.get_param('~out_topic', '/cmd_vel_auto_raw')
        self.rate_hz = float(rospy.get_param('~rate_hz', 30.0))

        self.tfbuf = tf2_ros.Buffer(cache_time=rospy.Duration(10.0))
        self.tfl = tf2_ros.TransformListener(self.tfbuf)

        self.cmd_pub = rospy.Publisher(self.out_topic, Twist, queue_size=10)

        self.points = self.load_path(self.path_yaml)
        if len(self.points) < 5:
            raise RuntimeError('Path too short: %d points' % len(self.points))

        self.prev_v = 0.0
        rospy.loginfo("PurePursuit loaded %d points, publishing %s", len(self.points), self.out_topic)

    def load_path(self, path_yaml):
        with open(path_yaml, 'r') as f:
            data = yaml.safe_load(f) or {}

        pts = []

        # Accept either:
        #  A) points: [ {x:..., y:...}, ... ]
        #  B) waypoints: [ [x,y], [x,y], ... ]
        if 'points' in data and isinstance(data['points'], list):
            for p in data['points']:
                pts.append((float(p['x']), float(p['y'])))
            return pts

        if 'waypoints' in data and isinstance(data['waypoints'], list):
            for p in data['waypoints']:
                pts.append((float(p[0]), float(p[1])))
            return pts

        raise RuntimeError("YAML must contain either 'points:' or 'waypoints:'")

    def get_pose(self):
        tr = self.tfbuf.lookup_transform(self.frame_id, self.base_frame,
                                        rospy.Time(0), rospy.Duration(0.2))
        x = tr.transform.translation.x
        y = tr.transform.translation.y
        q = tr.transform.rotation

        siny = 2.0 * (q.w*q.z + q.x*q.y)
        cosy = 1.0 - 2.0 * (q.y*q.y + q.z*q.z)
        yaw = math.atan2(siny, cosy)
        return x, y, yaw

    def nearest_index(self, x, y):
        best_i = 0
        best_d = 1e18
        for i, (px, py) in enumerate(self.points):
            d = (px-x)*(px-x) + (py-y)*(py-y)
            if d < best_d:
                best_d = d
                best_i = i
        return best_i

    def lookahead_point(self, i0, x, y):
        n = len(self.points)
        i = i0
        for _ in range(n):
            px, py = self.points[i]
            if math.hypot(px-x, py-y) >= self.lookahead:
                return px, py
            i = (i + 1) % n
        return self.points[i0]

    def compute_cmd(self, x, y, yaw):
        i0 = self.nearest_index(x, y)
        gx, gy = self.lookahead_point(i0, x, y)

        dx = gx - x
        dy = gy - y

        # goal in robot frame
        xr = math.cos(-yaw)*dx - math.sin(-yaw)*dy
        yr = math.sin(-yaw)*dx + math.cos(-yaw)*dy

        Ld = math.hypot(xr, yr)
        if Ld < 1e-3:
            return 0.0, 0.0

        kappa = 2.0 * yr / (Ld*Ld)

        if abs(kappa) > 1e-6:
            v = math.sqrt(max(0.0, self.a_lat_max / max(1e-6, abs(kappa))))
        else:
            v = self.v_max

        v = clamp(v, self.v_min, self.v_max)

        dv = clamp(v - self.prev_v,
                   -self.accel_limit/self.rate_hz,
                   self.accel_limit/self.rate_hz)
        v = self.prev_v + dv
        self.prev_v = v

        w = v * kappa
        return v, w

    def spin(self):
        r = rospy.Rate(self.rate_hz)
        while not rospy.is_shutdown():
            try:
                x, y, yaw = self.get_pose()
                v, w = self.compute_cmd(x, y, yaw)

                cmd = Twist()
                cmd.linear.x = v
                cmd.angular.z = w
                self.cmd_pub.publish(cmd)

            except Exception as e:
                self.cmd_pub.publish(Twist())
                rospy.logwarn_throttle(1.0, "PurePursuit waiting: %s" % str(e))

            r.sleep()


if __name__ == '__main__':
    rospy.init_node('pure_pursuit')
    node = PurePursuit()
    node.spin()

