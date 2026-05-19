#!/usr/bin/env python3

import argparse
import csv
from pathlib import Path

import cv2
import numpy as np
from scipy.spatial.transform import Rotation as R


def load_sync_rows(path, start_frame, end_frame, max_dt):
    rows = []
    with open(path, "r") as f:
        reader = csv.DictReader(f)
        for row in reader:
            frame_id = int(row["frame_id"])
            dt = float(row["dt"])

            if frame_id < start_frame or frame_id > end_frame:
                continue

            if dt > max_dt:
                print(f"Skipping frame {frame_id}: dt={dt:.6f} > max_dt={max_dt}")
                continue

            rows.append(row)

    return rows


def pose_to_matrix(row):
    tx = float(row["tx"])
    ty = float(row["ty"])
    tz = float(row["tz"])
    qx = float(row["qx"])
    qy = float(row["qy"])
    qz = float(row["qz"])
    qw = float(row["qw"])

    T = np.eye(4, dtype=np.float32)
    T[:3, :3] = R.from_quat([qx, qy, qz, qw]).as_matrix().astype(np.float32)
    T[:3, 3] = np.array([tx, ty, tz], dtype=np.float32)

    return T


def backproject_frame(rgb_bgr, depth, K, T_w_c, stride, max_depth):
    h, w = depth.shape

    rgb_bgr = cv2.resize(rgb_bgr, (w, h), interpolation=cv2.INTER_AREA)

    fx = float(K[0, 0])
    fy = float(K[1, 1])
    cx = float(K[0, 2])
    cy = float(K[1, 2])

    us, vs = np.meshgrid(
        np.arange(0, w, stride),
        np.arange(0, h, stride),
    )

    z = depth[vs, us].astype(np.float32)
    valid = np.isfinite(z) & (z > 0)

    if max_depth is not None:
        valid &= z < max_depth

    us = us[valid].astype(np.float32)
    vs = vs[valid].astype(np.float32)
    z = z[valid]

    x = (us - cx) * z / fx
    y = (vs - cy) * z / fy

    pts_c = np.stack([x, y, z], axis=1)
    pts_c_h = np.concatenate(
        [pts_c, np.ones((pts_c.shape[0], 1), dtype=np.float32)],
        axis=1,
    )

    pts_w = (T_w_c @ pts_c_h.T).T[:, :3]

    colors_bgr = rgb_bgr[vs.astype(np.int32), us.astype(np.int32)]
    colors_rgb = colors_bgr[:, ::-1]

    return pts_w, colors_rgb


def save_ply(path, points, colors):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)

    with path.open("w") as f:
        f.write("ply\n")
        f.write("format ascii 1.0\n")
        f.write(f"element vertex {points.shape[0]}\n")
        f.write("property float x\n")
        f.write("property float y\n")
        f.write("property float z\n")
        f.write("property uchar red\n")
        f.write("property uchar green\n")
        f.write("property uchar blue\n")
        f.write("end_header\n")

        for p, c in zip(points, colors):
            f.write(
                f"{p[0]:.6f} {p[1]:.6f} {p[2]:.6f} "
                f"{int(c[0])} {int(c[1])} {int(c[2])}\n"
            )


def main():
    parser = argparse.ArgumentParser(
        description="Fuse DA3 depth chunk with SVO poses into one colored PLY."
    )
    parser.add_argument("--rgb_dir", required=True)
    parser.add_argument("--da3_npz", required=True)
    parser.add_argument("--sync_csv", required=True)
    parser.add_argument("--output", required=True)
    parser.add_argument("--start_frame", type=int, default=1)
    parser.add_argument("--end_frame", type=int, default=15)
    parser.add_argument("--stride", type=int, default=6)
    parser.add_argument("--max_dt", type=float, default=0.20)
    parser.add_argument("--max_depth", type=float, default=None)
    args = parser.parse_args()

    rgb_dir = Path(args.rgb_dir)
    da3 = np.load(args.da3_npz)

    depth_all = da3["depth"]
    K_all = da3["intrinsics"]

    rows = load_sync_rows(
        args.sync_csv,
        start_frame=args.start_frame,
        end_frame=args.end_frame,
        max_dt=args.max_dt,
    )

    print(f"Using {len(rows)} synchronized frames")

    all_points = []
    all_colors = []

    for row in rows:
        frame_id = int(row["frame_id"])
        filename = row["filename"]
        rgb_path = rgb_dir / filename

        rgb = cv2.imread(str(rgb_path), cv2.IMREAD_COLOR)
        if rgb is None:
            raise FileNotFoundError(rgb_path)

        depth = depth_all[frame_id]
        K = K_all[frame_id]
        T_w_c = pose_to_matrix(row)

        pts, cols = backproject_frame(
            rgb_bgr=rgb,
            depth=depth,
            K=K,
            T_w_c=T_w_c,
            stride=args.stride,
            max_depth=args.max_depth,
        )

        all_points.append(pts)
        all_colors.append(cols)

        print(f"Frame {frame_id}: {pts.shape[0]} points")

    points = np.concatenate(all_points, axis=0)
    colors = np.concatenate(all_colors, axis=0)

    save_ply(args.output, points, colors)

    print(f"Saved: {args.output}")
    print(f"Total points: {points.shape[0]}")


if __name__ == "__main__":
    main()
