#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
bag_to_raceline_yaml.py (ROS Melodic / Python2)

Converts a recorded rosbag lap into a raceline YAML for r2_raceline_pp.

Output YAML:
  frame_id: map
  points:
    - {x: ..., y: ...}
  waypoints:
    - [x, y]

So all downstream tools work (old + new).
"""

from __future__ import print_function
import os
import sys
import argparse
import yaml


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("-b", "--bag", required=True, help="Input rosbag path")
    ap.add_argument("-t", "--topic", default="/tf", help="Topic to sample (or /odom etc., depending on your bag)")
    ap.add_argument("-o", "--out", required=True, help="Output YAML path")
    ap.add_argument("--frame-id", default="map", help="frame_id to store in YAML")
    ap.add_argument("--max-points", type=int, default=0, help="0 = no limit")
    args = ap.parse_args()

    bag_path = os.path.expanduser(args.bag)
    out_path = os.path.expanduser(args.out)

    if not os.path.isfile(bag_path):
        print("ERROR: bag not found:", bag_path)
        return 2

    # Import rosbag lazily (keeps this script robust on quirky Jetson images)
    import rosbag

    points = []

    # NOTE:
    # Your actual extraction depends on what you recorded.
    # Many setups record /tf or /odom. This generic reader:
    # - If msg has pose.pose.position (nav_msgs/Odometry), use that
    # - Else if msg has transform.translation (tf2_msgs/TFMessage), skip unless you customize
    #
    # If your bag topic is /odom, set --topic /odom.
    #
    with rosbag.Bag(bag_path, "r") as bag:
        for topic, msg, t in bag.read_messages(topics=[args.topic]):
            x = None
            y = None

            # nav_msgs/Odometry shape
            if hasattr(msg, "pose") and hasattr(msg.pose, "pose") and hasattr(msg.pose.pose, "position"):
                x = msg.pose.pose.position.x
                y = msg.pose.pose.position.y

            if x is None or y is None:
                continue

            points.append({"x": float(x), "y": float(y)})

            if args.max_points > 0 and len(points) >= args.max_points:
                break

    if len(points) < 2:
        print("ERROR: extracted <2 points. Try a different --topic (often /odom).")
        return 3

    # Also store old-style list-of-lists (optional compatibility)
    waypoints = [[p["x"], p["y"]] for p in points]

    out_dir = os.path.dirname(out_path)
    if out_dir and not os.path.isdir(out_dir):
        os.makedirs(out_dir)

    data = {
        "frame_id": args.frame_id,
        "points": points,
        "waypoints": waypoints,
    }

    with open(out_path, "w") as f:
        yaml.safe_dump(data, f, default_flow_style=False)

    print("Wrote raceline:", out_path)
    print("Points:", len(points))
    return 0


if __name__ == "__main__":
    sys.exit(main())

