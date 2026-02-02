#!/usr/bin/env python2
# coding: utf-8
"""
Bag -> Raceline YAML converter (Python2, ROS1 Noetic/Melodic).

Reads (recommended):
  - /amcl_pose (geometry_msgs/PoseWithCovarianceStamped)  [MAP frame]

Writes:
  - raceline.yaml with:
      frame_id: map
      waypoints: [[x,y], ...]

Features:
  - Thinning by --every
  - Optional min distance filter (--min_dist)
  - Fixed-spacing resampling (--resample) for racing (recommended)

Usage:
  rosrun r2_raceline_pp bag_to_raceline_yaml_amcl.py \
    --bag ~/bags/lap1_map.bag \
    --pose /amcl_pose \
    --out ~/yahboomcar_ws/src/r2_raceline_pp/config/raceline.yaml \
    --every 1 \
    --min_dist 0.00 \
    --resample 0.05
"""

from __future__ import print_function
import argparse
import yaml
import math
import rosbag


def parse_args():
    ap = argparse.ArgumentParser()
    ap.add_argument("--bag", required=True, help="Input bag file")
    ap.add_argument("--pose", default="/amcl_pose", help="Pose topic (PoseWithCovarianceStamped)")
    ap.add_argument("--out", required=True, help="Output yaml file")
    ap.add_argument("--every", type=int, default=1, help="Keep every Nth message")
    ap.add_argument("--min_dist", type=float, default=0.00, help="Minimum distance between raw points (meters)")
    ap.add_argument("--frame_id", default="map", help="Frame id to write into yaml (should be map)")
    ap.add_argument("--resample", type=float, default=0.0,
                    help="If >0, resample path to fixed spacing (meters), e.g. 0.05")
    ap.add_argument("--close_loop", action="store_true",
                    help="If set, treat path as closed loop (connect end->start) before resampling")
    return ap.parse_args()


def dist(a, b):
    return math.hypot(b[0] - a[0], b[1] - a[1])


def dedupe_consecutive(points, eps=1e-6):
    if not points:
        return points
    out = [points[0]]
    for p in points[1:]:
        if dist(out[-1], p) > eps:
            out.append(p)
    return out


def resample_polyline(points, step, closed=False):
    """
    Resample a polyline at fixed spacing.

    points: list of (x,y), length >= 2
    step: spacing in meters
    closed: if True, connect last->first
    """
    if step <= 0.0:
        return points[:]

    pts = [(float(x), float(y)) for x, y in points]
    pts = dedupe_consecutive(pts)

    if len(pts) < 2:
        return pts

    if closed:
        # append first point to end to create closing segment
        pts = pts + [pts[0]]

    # Build cumulative arc length
    seg_lens = []
    cum = [0.0]
    total = 0.0
    for i in range(1, len(pts)):
        L = dist(pts[i - 1], pts[i])
        seg_lens.append(L)
        total += L
        cum.append(total)

    if total < step:
        return pts[:-1] if closed else pts

    # Sample distances
    n = int(math.floor(total / step))
    samples = [i * step for i in range(n + 1)]
    # Ensure final point for open polylines
    if not closed and (total - samples[-1]) > 1e-6:
        samples.append(total)

    out = []
    seg_idx = 0

    for s in samples:
        # Advance to the segment containing s
        while seg_idx < len(seg_lens) - 1 and cum[seg_idx + 1] < s:
            seg_idx += 1

        s0 = cum[seg_idx]
        s1 = cum[seg_idx + 1]
        p0 = pts[seg_idx]
        p1 = pts[seg_idx + 1]

        if s1 <= s0 + 1e-12:
            # zero-length segment (shouldn't happen after dedupe, but safe)
            out.append((p0[0], p0[1]))
            continue

        u = (s - s0) / (s1 - s0)
        x = p0[0] + u * (p1[0] - p0[0])
        y = p0[1] + u * (p1[1] - p0[1])
        out.append((x, y))

    if closed:
        # remove duplicated last sample (which will land near the start)
        if len(out) > 1 and dist(out[0], out[-1]) < 1e-6:
            out = out[:-1]

    return out


def main():
    args = parse_args()

    raw = []
    last = None
    count = 0

    with rosbag.Bag(args.bag, "r") as bag:
        for topic, msg, t in bag.read_messages(topics=[args.pose]):
            count += 1
            if args.every > 1 and (count % args.every) != 0:
                continue

            x = float(msg.pose.pose.position.x)
            y = float(msg.pose.pose.position.y)

            if last is not None and args.min_dist > 0.0:
                if math.hypot(x - last[0], y - last[1]) < args.min_dist:
                    continue

            raw.append((x, y))
            last = (x, y)

    if len(raw) < 10:
        print("ERROR: too few raw points (%d). Drive longer or lower --every." % len(raw))
        return 2

    pts = raw
    if args.resample and args.resample > 0.0:
        pts = resample_polyline(pts, args.resample, closed=args.close_loop)

    if len(pts) < 10:
        print("ERROR: too few output points (%d). Try smaller --resample." % len(pts))
        return 2

    # Round for YAML readability
    pts_out = [[round(p[0], 3), round(p[1], 3)] for p in pts]

    data = {"frame_id": args.frame_id, "waypoints": pts_out}

    with open(args.out, "w") as f:
        yaml.safe_dump(data, f, default_flow_style=False)

    print("Raw points: %d" % len(raw))
    if args.resample and args.resample > 0.0:
        print("Resampled points: %d (step=%.3f m, closed=%s)" % (len(pts), args.resample, str(bool(args.close_loop))))
    print("Wrote %d points to %s (frame_id=%s)" % (len(pts_out), args.out, args.frame_id))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
