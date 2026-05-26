#!/usr/bin/env python3
"""
Fuse DA3 depth maps into one global PLY using SVO poses directly.

This avoids using DA3 local extrinsics for global fusion.

Pipeline:
  for each chunk:
    load results.npz
    load frames.csv
    for each frame:
      match frame timestamp to nearest SVO pose
      backproject DA3 depth using DA3 intrinsics
      transform points using SVO pose
      append to global cloud

Expected structure:
  outputs/da3_v1_01_16f/
  ├── chunks/<chunk_name>/frames.csv
  ├── chunks/<chunk_name>/images/frame_xxxxxx.png
  ├── da3/<chunk_name>/exports/mini_npz/results.npz
  └── ...

SVO trajectory format:
  # timestamp tx ty tz qx qy qz qw
"""

from __future__ import annotations

import argparse
import csv
from pathlib import Path

import cv2
import numpy as np


def read_frames_csv(path: Path) -> list[dict]:
    with path.open(newline="") as f:
        return list(csv.DictReader(f))


def get_timestamp_seconds(row: dict) -> float:
    for key in [
        "timestamp",
        "timestamp_s",
        "timestamp_sec",
        "time",
        "time_s",
        "t",
        "timestamp_ns",
        "timestamp_nanoseconds",
    ]:
        if key in row and row[key] != "":
            value = float(row[key])

            # EuRoC nanoseconds
            if value > 1e15:
                return value * 1e-9

            # possible microseconds
            if value > 1e12:
                return value * 1e-6

            return value

    raise ValueError(f"No timestamp column found in row: {row}")


def get_frame_id(row: dict) -> int:
    for key in ["frame_id", "frame", "id", "local_id"]:
        if key in row and row[key] != "":
            return int(row[key])

    name = row.get("filename", "")
    digits = "".join(ch for ch in Path(name).stem if ch.isdigit())
    if digits:
        return int(digits)

    raise ValueError(f"No frame_id found in row: {row}")


def load_svo_tum(path: Path) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """
    Load SVO TUM trajectory.

    Returns:
      timestamps: N
      positions: Nx3
      quaternions: Nx4 as qx qy qz qw
    """
    timestamps = []
    positions = []
    quats = []

    with path.open() as f:
        for line in f:
            line = line.strip()

            if not line or line.startswith("#"):
                continue

            line = line.replace(",", " ")
            parts = line.split()

            try:
                values = [float(x) for x in parts]
            except ValueError:
                continue

            if len(values) < 8:
                continue

            timestamps.append(values[0])
            positions.append(values[1:4])
            quats.append(values[4:8])

    if not timestamps:
        raise RuntimeError(f"No SVO poses loaded from {path}")

    timestamps = np.asarray(timestamps, dtype=np.float64)
    positions = np.asarray(positions, dtype=np.float64)
    quats = np.asarray(quats, dtype=np.float64)

    order = np.argsort(timestamps)

    return timestamps[order], positions[order], quats[order]


def normalize_quat(q: np.ndarray) -> np.ndarray:
    n = np.linalg.norm(q)
    if n < 1e-12:
        raise ValueError("Invalid zero quaternion")
    return q / n


def quat_xyzw_to_rotmat(q: np.ndarray) -> np.ndarray:
    """
    q = [qx, qy, qz, qw]
    Returns rotation matrix.
    """
    qx, qy, qz, qw = normalize_quat(q)

    xx = qx * qx
    yy = qy * qy
    zz = qz * qz
    xy = qx * qy
    xz = qx * qz
    yz = qy * qz
    wx = qw * qx
    wy = qw * qy
    wz = qw * qz

    R = np.array(
        [
            [1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz), 2.0 * (xz + wy)],
            [2.0 * (xy + wz), 1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx)],
            [2.0 * (xz - wy), 2.0 * (yz + wx), 1.0 - 2.0 * (xx + yy)],
        ],
        dtype=np.float64,
    )

    return R


def nearest_svo_pose(
    query_t: float,
    timestamps: np.ndarray,
    positions: np.ndarray,
    quats: np.ndarray,
    max_dt: float,
) -> tuple[np.ndarray | None, np.ndarray | None, float]:
    idx = int(np.searchsorted(timestamps, query_t))

    candidates = []
    if idx < len(timestamps):
        candidates.append(idx)
    if idx > 0:
        candidates.append(idx - 1)

    if not candidates:
        return None, None, float("inf")

    best = min(candidates, key=lambda i: abs(timestamps[i] - query_t))
    dt = abs(timestamps[best] - query_t)

    if dt > max_dt:
        return None, None, dt

    return positions[best], quats[best], dt


def transform_points(R: np.ndarray, t: np.ndarray, pts: np.ndarray) -> np.ndarray:
    return (R @ pts.T).T + t


def save_ascii_ply(path: Path, points: np.ndarray, colors: np.ndarray | None) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)

    with path.open("w") as f:
        f.write("ply\n")
        f.write("format ascii 1.0\n")
        f.write(f"element vertex {len(points)}\n")
        f.write("property float x\n")
        f.write("property float y\n")
        f.write("property float z\n")

        if colors is not None:
            f.write("property uchar red\n")
            f.write("property uchar green\n")
            f.write("property uchar blue\n")

        f.write("end_header\n")

        if colors is None:
            for p in points:
                f.write(f"{p[0]:.6f} {p[1]:.6f} {p[2]:.6f}\n")
        else:
            for p, c in zip(points, colors):
                f.write(
                    f"{p[0]:.6f} {p[1]:.6f} {p[2]:.6f} "
                    f"{int(c[0])} {int(c[1])} {int(c[2])}\n"
                )


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)

    parser.add_argument("--output_root", required=True)
    parser.add_argument("--svo_traj", required=True)
    parser.add_argument("--output", default=None)

    parser.add_argument("--start_chunk", type=int, default=0)
    parser.add_argument("--max_chunks", type=int, default=10**9)
    parser.add_argument("--max_time_diff", type=float, default=0.03)

    parser.add_argument("--stride", type=int, default=4)
    parser.add_argument("--min_depth", type=float, default=0.2)
    parser.add_argument("--max_depth", type=float, default=8.0)
    parser.add_argument("--conf_percentile", type=float, default=10.0)

    parser.add_argument("--skip_duplicate_frames", action="store_true")
    parser.add_argument("--max_points", type=int, default=0)

    parser.add_argument(
        "--pose_mode",
        choices=["t_w_c", "t_c_w"],
        default="t_w_c",
        help=(
            "Interpret SVO pose as camera-to-world t_w_c by default. "
            "Use t_c_w if the output looks inverted/exploded."
        ),
    )

    parser.add_argument(
        "--depth_scale",
        type=float,
        default=1.0,
        help="Multiply DA3 depth by this value before fusion.",
    )

    args = parser.parse_args()

    output_root = Path(args.output_root)
    chunks_dir = output_root / "chunks"
    da3_dir = output_root / "da3"

    output_path = Path(args.output) if args.output else output_root / "merged_depth_svo_direct.ply"

    svo_t, svo_pos, svo_quat = load_svo_tum(Path(args.svo_traj))

    all_chunks = sorted([p for p in chunks_dir.glob("chunk_*") if p.is_dir()])
    chunks = all_chunks[args.start_chunk : args.start_chunk + args.max_chunks]

    print(f"Output root:          {output_root}")
    print(f"Chunks dir:           {chunks_dir}")
    print(f"DA3 dir:              {da3_dir}")
    print(f"SVO traj:             {args.svo_traj}")
    print(f"SVO poses:            {len(svo_t)}")
    print(f"SVO time range:       {svo_t[0]:.9f} to {svo_t[-1]:.9f}")
    print(f"Selected chunks:      {len(chunks)}")
    print(f"Output:               {output_path}")
    print(f"Pose mode:            {args.pose_mode}")
    print(f"Depth scale:          {args.depth_scale}")
    print(f"Stride:               {args.stride}")
    print(f"Depth range:          {args.min_depth} to {args.max_depth}")
    print(f"Conf percentile:      {args.conf_percentile}")
    print(f"Skip duplicates:      {args.skip_duplicate_frames}")
    print()

    all_points = []
    all_colors = []

    used_frame_ids: set[int] = set()

    total_frames_used = 0
    total_frames_skipped_time = 0
    total_frames_skipped_duplicate = 0
    total_chunks_skipped = 0

    for chunk_path in chunks:
        chunk_name = chunk_path.name
        print("============================================================")
        print(f"Chunk: {chunk_name}")

        frames_csv = chunk_path / "frames.csv"
        image_dir = chunk_path / "images"
        da3_npz = da3_dir / chunk_name / "exports" / "mini_npz" / "results.npz"

        if not frames_csv.exists():
            print(f"[SKIP] missing frames.csv: {frames_csv}")
            total_chunks_skipped += 1
            continue

        if not image_dir.exists():
            print(f"[SKIP] missing image dir: {image_dir}")
            total_chunks_skipped += 1
            continue

        if not da3_npz.exists():
            print(f"[SKIP] missing DA3 npz: {da3_npz}")
            total_chunks_skipped += 1
            continue

        rows = read_frames_csv(frames_csv)
        data = np.load(da3_npz)

        depth = data["depth"].astype(np.float64) * args.depth_scale
        intrinsics = data["intrinsics"].astype(np.float64)
        conf = data["conf"] if "conf" in data else None

        if len(rows) != depth.shape[0]:
            print(f"[SKIP] row/depth mismatch: {len(rows)} vs {depth.shape[0]}")
            total_chunks_skipped += 1
            continue

        chunk_frames_used = 0
        chunk_points = 0
        dts = []

        for i, row in enumerate(rows):
            frame_id = get_frame_id(row)

            if args.skip_duplicate_frames and frame_id in used_frame_ids:
                total_frames_skipped_duplicate += 1
                continue

            t_frame = get_timestamp_seconds(row)

            pos, quat, dt = nearest_svo_pose(
                t_frame,
                svo_t,
                svo_pos,
                svo_quat,
                args.max_time_diff,
            )

            if pos is None or quat is None:
                total_frames_skipped_time += 1
                continue

            z_img = depth[i]
            h, w = z_img.shape

            K = intrinsics[i].copy()

            # If K belongs to original image size and depth was resized by DA3,
            # DA3 mini_npz usually stores K already corresponding to depth size.
            # So we trust K as exported by DA3.
            fx, fy = K[0, 0], K[1, 1]
            cx, cy = K[0, 2], K[1, 2]

            us, vs = np.meshgrid(
                np.arange(0, w, args.stride),
                np.arange(0, h, args.stride),
            )

            z = z_img[vs, us]

            valid = np.isfinite(z) & (z >= args.min_depth) & (z <= args.max_depth)

            if conf is not None:
                conf_img = conf[i]
                conf_sample = conf_img[vs, us]
                threshold = np.percentile(conf_img[np.isfinite(conf_img)], args.conf_percentile)
                valid &= np.isfinite(conf_sample) & (conf_sample >= threshold)

            if not np.any(valid):
                continue

            u = us[valid].astype(np.float64)
            v = vs[valid].astype(np.float64)
            z = z[valid].astype(np.float64)

            x = (u - cx) * z / fx
            y = (v - cy) * z / fy

            pts_c = np.stack([x, y, z], axis=1)

            R = quat_xyzw_to_rotmat(quat)
            t = pos.astype(np.float64)

            if args.pose_mode == "t_w_c":
                pts_w = transform_points(R, t, pts_c)
            else:
                # If pose is world-to-camera, invert it.
                pts_w = (R.T @ (pts_c - t).T).T

            rgb = cv2.imread(str(image_dir / row["filename"]), cv2.IMREAD_COLOR)
            if rgb is None:
                raise FileNotFoundError(image_dir / row["filename"])

            rgb = cv2.resize(rgb, (w, h), interpolation=cv2.INTER_AREA)
            colors = rgb[v.astype(np.int32), u.astype(np.int32), ::-1]

            all_points.append(pts_w)
            all_colors.append(colors)

            used_frame_ids.add(frame_id)
            total_frames_used += 1
            chunk_frames_used += 1
            chunk_points += len(pts_w)
            dts.append(dt)

            if args.max_points > 0:
                current_points = sum(len(p) for p in all_points)
                if current_points >= args.max_points:
                    print(f"[STOP] reached max_points={args.max_points}")
                    break

        if dts:
            print(f"Frames used: {chunk_frames_used}")
            print(f"Points:      {chunk_points}")
            print(f"Mean dt:     {np.mean(dts):.9f} s")
            print(f"Max dt:      {np.max(dts):.9f} s")
        else:
            print("Frames used: 0")

        if args.max_points > 0:
            current_points = sum(len(p) for p in all_points)
            if current_points >= args.max_points:
                break

    if not all_points:
        raise RuntimeError("No points fused. Check SVO timestamps and chunk timestamps.")

    points = np.concatenate(all_points, axis=0)
    colors = np.concatenate(all_colors, axis=0)

    if args.max_points > 0 and len(points) > args.max_points:
        points = points[: args.max_points]
        colors = colors[: args.max_points]

    save_ascii_ply(output_path, points, colors)

    print("============================================================")
    print("Direct SVO-depth fusion complete.")
    print(f"Saved:                    {output_path}")
    print(f"Total points:             {len(points)}")
    print(f"Total frames used:        {total_frames_used}")
    print(f"Frames skipped by time:   {total_frames_skipped_time}")
    print(f"Duplicate frames skipped: {total_frames_skipped_duplicate}")
    print(f"Chunks skipped:           {total_chunks_skipped}")


if __name__ == "__main__":
    main()