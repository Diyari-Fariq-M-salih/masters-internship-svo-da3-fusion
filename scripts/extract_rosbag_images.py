#!/usr/bin/env python3

import argparse
import csv
from pathlib import Path

import cv2
import rosbag
from cv_bridge import CvBridge


def main():
    parser = argparse.ArgumentParser(description="Extract images from a ROS bag topic.")
    parser.add_argument("--bag", required=True, help="Input ROS bag path")
    parser.add_argument("--topic", default="/cam0/image_raw", help="Image topic")
    parser.add_argument("--output_dir", required=True, help="Output image directory")
    parser.add_argument("--timestamps", required=True, help="Output CSV timestamp file")
    parser.add_argument("--max_frames", type=int, default=100, help="Maximum frames to extract")
    parser.add_argument("--every_n", type=int, default=1, help="Extract every Nth image message")
    parser.add_argument("--prefix", default="frame", help="Output image prefix")
    args = parser.parse_args()

    output_dir = Path(args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)

    timestamps_path = Path(args.timestamps)
    timestamps_path.parent.mkdir(parents=True, exist_ok=True)

    bridge = CvBridge()
    extracted = 0
    seen = 0

    with rosbag.Bag(args.bag, "r") as bag, timestamps_path.open("w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["frame_id", "timestamp", "filename", "topic"])

        for topic, msg, _ in bag.read_messages(topics=[args.topic]):
            if seen % args.every_n != 0:
                seen += 1
                continue

            stamp = msg.header.stamp.to_sec()
            filename = f"{args.prefix}_{extracted:06d}.png"
            out_path = output_dir / filename

            cv_img = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            cv2.imwrite(str(out_path), cv_img)

            writer.writerow([extracted, f"{stamp:.9f}", filename, topic])

            extracted += 1
            seen += 1

            if extracted >= args.max_frames:
                break

    print(f"Extracted {extracted} frames to {output_dir}")
    print(f"Wrote timestamps to {timestamps_path}")


if __name__ == "__main__":
    main()
