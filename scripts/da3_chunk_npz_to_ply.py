#!/usr/bin/env python3
"""Convert a DA3 mini_npz chunk to a local colored PLY for inspection."""

from __future__ import annotations

import argparse
import csv
from pathlib import Path

import cv2
import numpy as np


def load_frame_rows(path: str | Path) -> list[dict]:
    with Path(path).open(newline="") as f:
        return list(csv.DictReader(f))


def transform_points(T: np.ndarray, pts: np.ndarray) -> np.ndarray:
    return (T[:3, :3] @ pts.T).T + T[:3, 3]


def extrinsic_to_t_w_c(extrinsic_3x4: np.ndarray, mode: str) -> np.ndarray:
    T = np.eye(4, dtype=np.float64)
    T[:3, :] = extrinsic_3x4.astype(np.float64)
    if mode == "as_t_w_c":
        return T
    if mode == "invert":
        return np.linalg.inv(T)
    raise ValueError(f"Unknown extrinsic mode: {mode}")


def save_ascii_ply(path: str | Path, points: np.ndarray, colors: np.ndarray) -> None:
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w") as f:
        f.write("ply\nformat ascii 1.0\n")
        f.write(f"element vertex {len(points)}\n")
        f.write("property float x\nproperty float y\nproperty float z\n")
        f.write("property uchar red\nproperty uchar green\nproperty uchar blue\n")
        f.write("end_header\n")
        for p, c in zip(points, colors):
            f.write(
                f"{p[0]:.6f} {p[1]:.6f} {p[2]:.6f} "
                f"{int(c[0])} {int(c[1])} {int(c[2])}\n"
            )


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--da3_npz", required=True)
    parser.add_argument("--frames_csv", required=True)
    parser.add_argument("--image_dir", required=True)
    parser.add_argument("--output", required=True)
    parser.add_argument("--stride", type=int, default=6)
    parser.add_argument("--min_depth", type=float, default=0.2)
    parser.add_argument("--max_depth", type=float, default=8.0)
    parser.add_argument("--conf_percentile", type=float, default=25.0)
    parser.add_argument("--extrinsic_mode", choices=["as_t_w_c", "invert"], default="invert")
    args = parser.parse_args()

    data = np.load(args.da3_npz)
    depth = data["depth"]
    conf = data["conf"] if "conf" in data else None
    intrinsics = data["intrinsics"]
    extrinsics = data["extrinsics"]
    rows = load_frame_rows(args.frames_csv)
    if len(rows) != depth.shape[0]:
        raise ValueError(f"frames_csv has {len(rows)} rows but depth has {depth.shape[0]} frames")

    all_points = []
    all_colors = []
    for idx, row in enumerate(rows):
        rgb = cv2.imread(str(Path(args.image_dir) / row["filename"]), cv2.IMREAD_COLOR)
        if rgb is None:
            raise FileNotFoundError(Path(args.image_dir) / row["filename"])
        z_img = depth[idx].astype(np.float64)
        h, w = z_img.shape
        rgb = cv2.resize(rgb, (w, h), interpolation=cv2.INTER_AREA)
        K = intrinsics[idx].astype(np.float64)
        us, vs = np.meshgrid(np.arange(0, w, args.stride), np.arange(0, h, args.stride))
        z = z_img[vs, us]
        valid = np.isfinite(z) & (z >= args.min_depth) & (z <= args.max_depth)
        if conf is not None:
            c = conf[idx][vs, us]
            threshold = np.percentile(conf[idx], args.conf_percentile)
            valid &= np.isfinite(c) & (c >= threshold)
        u = us[valid].astype(np.float64)
        v = vs[valid].astype(np.float64)
        z = z[valid]
        x = (u - K[0, 2]) * z / K[0, 0]
        y = (v - K[1, 2]) * z / K[1, 1]
        pts_c = np.stack([x, y, z], axis=1)
        T_w_c = extrinsic_to_t_w_c(extrinsics[idx], args.extrinsic_mode)
        pts_w = transform_points(T_w_c, pts_c)
        colors = rgb[v.astype(np.int32), u.astype(np.int32), ::-1]
        all_points.append(pts_w)
        all_colors.append(colors)

    points = np.concatenate(all_points, axis=0)
    colors = np.concatenate(all_colors, axis=0)
    save_ascii_ply(args.output, points, colors)
    print(f"Saved {args.output}")
    print(f"Points: {len(points)}")
    print(f"Frames: {len(rows)}")
    print(f"Extrinsic mode: {args.extrinsic_mode}")


if __name__ == "__main__":
    main()
