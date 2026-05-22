#!/usr/bin/env python3
"""
Prepare sampled TUM RGB-D frames and overlapping DA3 input chunks.

The output intentionally mirrors the EuRoC helper outputs enough that existing
DA3 chunk scripts can reuse the generated rgb directory and sync CSV files.
"""

import argparse
import csv
import shutil
from bisect import bisect_left
from pathlib import Path


def load_assoc_file(path, expected_cols):
    rows = []
    with Path(path).open("r") as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            vals = line.split()
            if len(vals) < expected_cols:
                continue
            rows.append(vals)
    return rows


def nearest_by_time(timestamp, rows):
    times = [float(row[0]) for row in rows]
    idx = bisect_left(times, timestamp)
    candidates = []
    if idx < len(rows):
        candidates.append(idx)
    if idx > 0:
        candidates.append(idx - 1)
    if not candidates:
        raise ValueError("No rows to associate.")
    best = min(candidates, key=lambda i: abs(float(rows[i][0]) - timestamp))
    return rows[best], abs(float(rows[best][0]) - timestamp)


def associate(dataset_dir, max_depth_dt, max_gt_dt):
    dataset_dir = Path(dataset_dir)
    rgb_rows = load_assoc_file(dataset_dir / "rgb.txt", expected_cols=2)
    depth_rows = load_assoc_file(dataset_dir / "depth.txt", expected_cols=2)
    gt_rows = load_assoc_file(dataset_dir / "groundtruth.txt", expected_cols=8)

    associated = []
    for rgb in rgb_rows:
        rgb_time = float(rgb[0])
        depth, depth_dt = nearest_by_time(rgb_time, depth_rows)
        gt, gt_dt = nearest_by_time(rgb_time, gt_rows)
        if depth_dt > max_depth_dt or gt_dt > max_gt_dt:
            continue

        associated.append(
            {
                "rgb_timestamp": rgb_time,
                "rgb_relpath": rgb[1],
                "depth_timestamp": float(depth[0]),
                "depth_relpath": depth[1],
                "depth_dt": depth_dt,
                "gt_timestamp": float(gt[0]),
                "gt_dt": gt_dt,
                "tx": float(gt[1]),
                "ty": float(gt[2]),
                "tz": float(gt[3]),
                "qx": float(gt[4]),
                "qy": float(gt[5]),
                "qz": float(gt[6]),
                "qw": float(gt[7]),
            }
        )

    return associated


def parse_chunk(text):
    try:
        start, end = text.split(":")
        return int(start), int(end)
    except ValueError as exc:
        raise argparse.ArgumentTypeError(
            f"Chunk must be START:END, got {text!r}"
        ) from exc


def write_sync_csv(path, rows):
    with Path(path).open("w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(
            [
                "frame_id",
                "filename",
                "frame_timestamp",
                "pose_index",
                "pose_timestamp",
                "dt",
                "tx",
                "ty",
                "tz",
                "qx",
                "qy",
                "qz",
                "qw",
                "depth_timestamp",
                "depth_filename",
                "depth_dt",
                "original_rgb_filename",
            ]
        )
        for row in rows:
            writer.writerow(
                [
                    row["frame_id"],
                    row["filename"],
                    f"{row['rgb_timestamp']:.9f}",
                    row["frame_id"],
                    f"{row['gt_timestamp']:.9f}",
                    f"{row['gt_dt']:.9f}",
                    f"{row['tx']:.9f}",
                    f"{row['ty']:.9f}",
                    f"{row['tz']:.9f}",
                    f"{row['qx']:.9f}",
                    f"{row['qy']:.9f}",
                    f"{row['qz']:.9f}",
                    f"{row['qw']:.9f}",
                    f"{row['depth_timestamp']:.9f}",
                    row["depth_filename"],
                    f"{row['depth_dt']:.9f}",
                    row["original_rgb_filename"],
                ]
            )


def main():
    parser = argparse.ArgumentParser(
        description="Prepare sampled TUM RGB-D frames and DA3 chunk folders."
    )
    parser.add_argument("--dataset_dir", required=True)
    parser.add_argument("--output_root", required=True)
    parser.add_argument("--sample_stride", type=int, default=10)
    parser.add_argument("--max_depth_dt", type=float, default=0.03)
    parser.add_argument("--max_gt_dt", type=float, default=0.03)
    parser.add_argument("--chunk", action="append", type=parse_chunk, default=[])
    args = parser.parse_args()

    dataset_dir = Path(args.dataset_dir)
    output_root = Path(args.output_root)
    rgb_dir = output_root / "rgb"
    depth_dir = output_root / "depth"
    rgb_dir.mkdir(parents=True, exist_ok=True)
    depth_dir.mkdir(parents=True, exist_ok=True)

    associated = associate(dataset_dir, args.max_depth_dt, args.max_gt_dt)
    sampled = associated[:: args.sample_stride]

    prepared = []
    for frame_id, row in enumerate(sampled):
        rgb_name = f"frame_{frame_id:06d}.png"
        depth_name = f"frame_{frame_id:06d}.png"
        shutil.copy2(dataset_dir / row["rgb_relpath"], rgb_dir / rgb_name)
        shutil.copy2(dataset_dir / row["depth_relpath"], depth_dir / depth_name)

        out_row = dict(row)
        out_row["frame_id"] = frame_id
        out_row["filename"] = rgb_name
        out_row["depth_filename"] = depth_name
        out_row["original_rgb_filename"] = row["rgb_relpath"]
        prepared.append(out_row)

    write_sync_csv(output_root / "sync_all.csv", prepared)

    for start, end in args.chunk:
        if start < 0 or end >= len(prepared) or start > end:
            raise ValueError(
                f"Invalid chunk {start}:{end}; prepared frame range is 0:{len(prepared)-1}."
            )

        chunk_rgb_dir = output_root / f"rgb_{start:03d}_{end:03d}"
        chunk_rgb_dir.mkdir(parents=True, exist_ok=True)
        for frame_id in range(start, end + 1):
            row = prepared[frame_id]
            shutil.copy2(rgb_dir / row["filename"], chunk_rgb_dir / row["filename"])
        write_sync_csv(output_root / f"sync_{start:03d}_{end:03d}.csv", prepared[: end + 1])

    print(f"Dataset: {dataset_dir}")
    print(f"Associated RGB-D-GT frames: {len(associated)}")
    print(f"Sample stride: {args.sample_stride}")
    print(f"Prepared sampled frames: {len(prepared)}")
    print(f"Output root: {output_root}")
    for start, end in args.chunk:
        print(f"Chunk {start:03d}-{end:03d}: {output_root / f'rgb_{start:03d}_{end:03d}'}")


if __name__ == "__main__":
    main()
