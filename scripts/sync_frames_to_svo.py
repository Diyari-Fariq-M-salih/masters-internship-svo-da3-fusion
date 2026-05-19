#!/usr/bin/env python3

import argparse
import csv
from pathlib import Path

import numpy as np


def load_frame_timestamps(path):
    rows = []
    with open(path, "r") as f:
        reader = csv.DictReader(f)
        for row in reader:
            rows.append({
                "frame_id": int(row["frame_id"]),
                "timestamp": float(row["timestamp"]),
                "filename": row["filename"],
            })
    return rows


def load_tum(path):
    poses = []
    with open(path, "r") as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            vals = line.split()
            if len(vals) != 8:
                continue
            poses.append([float(v) for v in vals])
    return np.array(poses, dtype=np.float64)


def nearest_pose(timestamp, poses):
    pose_times = poses[:, 0]
    idx = int(np.argmin(np.abs(pose_times - timestamp)))
    dt = abs(pose_times[idx] - timestamp)
    return idx, dt, poses[idx]


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--frames_csv", required=True)
    parser.add_argument("--svo_tum", required=True)
    parser.add_argument("--output", required=True)
    parser.add_argument("--max_frames", type=int, default=16)
    parser.add_argument("--max_dt", type=float, default=0.10)
    args = parser.parse_args()

    frames = load_frame_timestamps(args.frames_csv)[: args.max_frames]
    poses = load_tum(args.svo_tum)

    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)

    kept = 0
    with output.open("w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow([
            "frame_id",
            "filename",
            "frame_timestamp",
            "pose_index",
            "pose_timestamp",
            "dt",
            "tx", "ty", "tz",
            "qx", "qy", "qz", "qw",
        ])

        for fr in frames:
            pose_idx, dt, pose = nearest_pose(fr["timestamp"], poses)

            if dt > args.max_dt:
                print(
                    f"WARNING: frame {fr['frame_id']} nearest pose dt={dt:.6f}s "
                    f"> max_dt={args.max_dt}s"
                )

            writer.writerow([
                fr["frame_id"],
                fr["filename"],
                f"{fr['timestamp']:.9f}",
                pose_idx,
                f"{pose[0]:.9f}",
                f"{dt:.9f}",
                f"{pose[1]:.9f}",
                f"{pose[2]:.9f}",
                f"{pose[3]:.9f}",
                f"{pose[4]:.9f}",
                f"{pose[5]:.9f}",
                f"{pose[6]:.9f}",
                f"{pose[7]:.9f}",
            ])
            kept += 1

    print(f"Wrote {kept} synchronized rows to {output}")


if __name__ == "__main__":
    main()
