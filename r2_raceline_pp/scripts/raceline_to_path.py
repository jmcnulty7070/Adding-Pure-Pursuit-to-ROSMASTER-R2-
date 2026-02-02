#!/usr/bin/env python
# -*- coding: utf-8 -*-
# r2_raceline_pp/scripts/raceline_to_path.py

import rospy
import yaml
import math
from geometry_msgs.msg import PoseStamped, Quaternion
from nav_msgs.msg import Path


def yaw_to_quat(yaw):
    q = Quaternion()
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(yaw * 0.5)
    q.w = math.cos(yaw * 0.5)
    return q


class RacelineToPath(object):
    def __init__(self):
        # Params (launch can override)
        self.frame_id_param = rospy.get_param("~frame_id", "map")
        self.yaml_path = rospy.get_param("~path_yaml", "")
        self.topic = rospy.get_param("~topic", "/raceline_path")
        self.hz = float(rospy.get_param("~rate_hz", 1.0))

        if not self.yaml_path:
            raise RuntimeError("~path_yaml param is empty")

        # Build Path from YAML
        self.path_msg, self.frame_id = self.load_yaml_as_path(self.yaml_path, self.frame_id_param)

        self.pub = rospy.Publisher(self.topic, Path, queue_size=1, latch=True)
        rospy.loginfo("raceline_to_path: publishing %d poses on %s frame_id=%s",
                      len(self.path_msg.poses), self.topic, self.frame_id)

    def load_yaml_as_path(self, fname, fallback_frame_id):
        with open(fname, "r") as f:
            data = yaml.safe_load(f)

        # Decide frame_id: param wins unless param is empty
        yaml_frame = None
        if isinstance(data, dict):
            yaml_frame = data.get("frame_id", None)
        frame_id = fallback_frame_id if fallback_frame_id else (yaml_frame or "map")

        # Accept formats:
        # A) dict with "points": [ {x:.., y:.., yaw(optional)} ... ]
        # B) dict with "waypoints": [ [x,y], [x,y], ... ]
        # C) root list: either list of dicts or list of [x,y]
        pts = None

        if isinstance(data, dict):
            if "points" in data and isinstance(data["points"], list):
                pts = data["points"]
            elif "waypoints" in data and isinstance(data["waypoints"], list):
                pts = data["waypoints"]
            else:
                # If dict but unknown, try a common alias
                for k in ["path", "poses"]:
                    if k in data and isinstance(data[k], list):
                        pts = data[k]
                        break
        elif isinstance(data, list):
            pts = data

        if pts is None or len(pts) < 2:
            raise RuntimeError("YAML does not contain usable points/waypoints (need at least 2)")

        # Normalize into list of (x,y,yaw_optional)
        xy = []
        for p in pts:
            if isinstance(p, dict):
                # dict form: {x:..., y:..., yaw:...}
                x = float(p.get("x", 0.0))
                y = float(p.get("y", 0.0))
                yaw = float(p.get("yaw", 0.0)) if ("yaw" in p) else None
                xy.append((x, y, yaw))
            elif isinstance(p, (list, tuple)) and len(p) >= 2:
                x = float(p[0])
                y = float(p[1])
                xy.append((x, y, None))
            else:
                # skip junk silently
                continue

        if len(xy) < 2:
            raise RuntimeError("After parsing, not enough valid points")

        # If yaw missing, compute yaw from segment direction (nice in RViz)
        yaws = []
        for i in range(len(xy)):
            x, y, yaw = xy[i]
            if yaw is not None:
                yaws.append(yaw)
                continue
            if i < len(xy) - 1:
                x2, y2, _ = xy[i + 1]
                yaws.append(math.atan2((y2 - y), (x2 - x)))
            else:
                # last point: reuse previous yaw
                yaws.append(yaws[-1] if yaws else 0.0)

        # Build Path msg
        now = rospy.Time.now()
        path = Path()
        path.header.stamp = now
        path.header.frame_id = frame_id

        for i in range(len(xy)):
            x, y, _ = xy[i]
            yaw = yaws[i]

            ps = PoseStamped()
            ps.header.stamp = now
            ps.header.frame_id = frame_id
            ps.pose.position.x = x
            ps.pose.position.y = y
            ps.pose.position.z = 0.0
            ps.pose.orientation = yaw_to_quat(yaw)
            path.poses.append(ps)

        return path, frame_id

    def spin(self):
        r = rospy.Rate(self.hz)
        while not rospy.is_shutdown():
            self.path_msg.header.stamp = rospy.Time.now()
            self.pub.publish(self.path_msg)
            r.sleep()


if __name__ == "__main__":
    rospy.init_node("raceline_to_path")
    node = RacelineToPath()
    node.spin()

