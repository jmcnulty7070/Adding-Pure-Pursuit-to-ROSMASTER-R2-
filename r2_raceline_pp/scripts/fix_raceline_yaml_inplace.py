#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
fix_raceline_yaml_inplace.py  (Python2 / ROS Melodic)

Converts:
  frame_id: map
  waypoints:
    - [x, y]

Into:
  frame_id: map
  points:
    - {x: x, y: y}

Creates a .bak backup of the original file.
"""

from __future__ import print_function
import os
import sys
import yaml


def main():
    if len(sys.argv) < 2:
        print("USAGE: python fix_raceline_yaml_inplace.py <raceline.yaml>")
        return 2

    path = os.path.expanduser(sys.argv[1])
    if not os.path.isfile(path):
        print("ERROR: file not found:", path)
        return 3

    with open(path, "r") as f:
        data = yaml.safe_load(f)

    frame_id = data.get("frame_id", "map")
    wps = data.get("waypoints", None)

    if not isinstance(wps, list):
        print("ERROR: no 'waypoints' list found")
        return 4

    points = []
    for p in wps:
        if isinstance(p, (list, tuple)) and len(p) >= 2:
            points.append({"x": float(p[0]), "y": float(p[1])})

    if len(points) < 2:
        print("ERROR: not enough valid points")
        return 5

    out = {
        "frame_id": frame_id,
        "points": points
    }

    bak = path + ".bak"
    if os.path.exists(bak):
        os.remove(bak)
    os.rename(path, bak)

    with open(path, "w") as f:
        yaml.safe_dump(out, f, default_flow_style=False)

    print("OK: YAML converted in place")
    print("Backup saved as:", bak)
    return 0


if __name__ == "__main__":
    sys.exit(main())
