#!/usr/bin/env python3

import argparse
from pathlib import Path

import rosbag


def pose_stamped_to_tum_line(msg):
    t = msg.header.stamp.to_sec()
    p = msg.pose.position
    q = msg.pose.orientation
    return f"{t:.9f} {p.x:.9f} {p.y:.9f} {p.z:.9f} {q.x:.9f} {q.y:.9f} {q.z:.9f} {q.w:.9f}"


def main():
    parser = argparse.ArgumentParser(
        description="Convert a ROS bag PoseStamped topic to TUM trajectory format."
    )
    parser.add_argument("--bag", required=True, help="Input ROS bag")
    parser.add_argument("--topic", default="/svo/pose_cam/0", help="PoseStamped topic")
    parser.add_argument("--output", required=True, help="Output TUM trajectory txt")
    args = parser.parse_args()

    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)

    count = 0
    with rosbag.Bag(args.bag, "r") as bag, output.open("w") as f:
        f.write("# timestamp tx ty tz qx qy qz qw\n")
        for topic, msg, _ in bag.read_messages(topics=[args.topic]):
            f.write(pose_stamped_to_tum_line(msg) + "\n")
            count += 1

    print(f"Wrote {count} poses to {output}")


if __name__ == "__main__":
    main()
