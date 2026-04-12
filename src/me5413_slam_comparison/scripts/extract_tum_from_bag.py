#!/usr/bin/env python3

import argparse
import os
import sys

import rosbag
from geometry_msgs.msg import PoseStamped


def tum_line(stamp, pose):
    p = pose.position
    q = pose.orientation
    return (
        f"{stamp.to_sec():.9f} "
        f"{p.x:.9f} {p.y:.9f} {p.z:.9f} "
        f"{q.x:.9f} {q.y:.9f} {q.z:.9f} {q.w:.9f}\n"
    )


def ensure_parent(path):
    parent = os.path.dirname(os.path.abspath(path))
    if parent:
        os.makedirs(parent, exist_ok=True)


def write_odometry_topic(bag, topic, output_path):
    count = 0
    ensure_parent(output_path)
    with open(output_path, "w", encoding="ascii") as handle:
        for _, msg, bag_stamp in bag.read_messages(topics=[topic]):
            if getattr(msg, "_type", "") != "nav_msgs/Odometry":
                raise TypeError(f"topic {topic} is not nav_msgs/Odometry")
            stamp = msg.header.stamp if msg.header.stamp.to_sec() > 0.0 else bag_stamp
            handle.write(tum_line(stamp, msg.pose.pose))
            count += 1
    return count


def write_path_topic(bag, topic, output_path):
    last_stamp = None
    count = 0
    ensure_parent(output_path)
    with open(output_path, "w", encoding="ascii") as handle:
        for _, msg, bag_stamp in bag.read_messages(topics=[topic]):
            if getattr(msg, "_type", "") != "nav_msgs/Path":
                raise TypeError(f"topic {topic} is not nav_msgs/Path")
            for pose_stamped in msg.poses:
                if getattr(pose_stamped, "_type", "") != "geometry_msgs/PoseStamped":
                    continue
                stamp = pose_stamped.header.stamp
                if stamp.to_sec() <= 0.0:
                    stamp = msg.header.stamp if msg.header.stamp.to_sec() > 0.0 else bag_stamp
                stamp_value = stamp.to_nsec()
                if last_stamp is not None and stamp_value <= last_stamp:
                    continue
                handle.write(tum_line(stamp, pose_stamped.pose))
                last_stamp = stamp_value
                count += 1
    return count


def main():
    parser = argparse.ArgumentParser(
        description="Extract nav_msgs/Odometry or nav_msgs/Path trajectories from a ROS bag into TUM format."
    )
    parser.add_argument("--bag", required=True, help="Input rosbag path")
    parser.add_argument("--odom-topic", action="append", default=[], help="Topic mapping in topic=output.tum form")
    parser.add_argument("--path-topic", action="append", default=[], help="Topic mapping in topic=output.tum form")
    args = parser.parse_args()

    jobs = []
    for spec in args.odom_topic:
        topic, output_path = spec.split("=", 1)
        jobs.append(("odom", topic, output_path))
    for spec in args.path_topic:
        topic, output_path = spec.split("=", 1)
        jobs.append(("path", topic, output_path))

    if not jobs:
        parser.error("at least one --odom-topic or --path-topic mapping is required")

    with rosbag.Bag(args.bag, "r") as bag:
        for kind, topic, output_path in jobs:
            if kind == "odom":
                count = write_odometry_topic(bag, topic, output_path)
            else:
                count = write_path_topic(bag, topic, output_path)
            print(f"{topic} -> {output_path} ({count} poses)")

    return 0


if __name__ == "__main__":
    sys.exit(main())
