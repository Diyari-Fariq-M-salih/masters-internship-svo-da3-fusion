#!/usr/bin/env python3

import argparse
from pathlib import Path

import cv2
import numpy as np


def save_ply(path, points, colors):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)

    with open(path, "w") as f:
        f.write("ply\n")
        f.write("format ascii 1.0\n")
        f.write(f"element vertex {len(points)}\n")
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
    parser = argparse.ArgumentParser()
    parser.add_argument("--rgb", required=True)
    parser.add_argument("--depth", required=True)
    parser.add_argument("--intrinsics", required=True)
    parser.add_argument("--output", required=True)
    parser.add_argument("--stride", type=int, default=4)
    args = parser.parse_args()

    rgb = cv2.imread(args.rgb, cv2.IMREAD_COLOR)
    if rgb is None:
        raise FileNotFoundError(args.rgb)

    depth = np.load(args.depth)[0]          # (1,H,W) -> (H,W)
    K = np.load(args.intrinsics)[0]         # (1,3,3) -> (3,3)

    h, w = depth.shape

    # Resize RGB to DA3 processed depth resolution
    rgb_resized = cv2.resize(rgb, (w, h), interpolation=cv2.INTER_AREA)

    fx, fy = K[0, 0], K[1, 1]
    cx, cy = K[0, 2], K[1, 2]

    us, vs = np.meshgrid(
        np.arange(0, w, args.stride),
        np.arange(0, h, args.stride),
    )

    z = depth[vs, us].astype(np.float32)
    valid = np.isfinite(z) & (z > 0)

    us = us[valid].astype(np.float32)
    vs = vs[valid].astype(np.float32)
    z = z[valid]

    x = (us - cx) * z / fx
    y = (vs - cy) * z / fy

    points = np.stack([x, y, z], axis=1)

    colors_bgr = rgb_resized[vs.astype(np.int32), us.astype(np.int32)]
    colors_rgb = colors_bgr[:, ::-1]

    save_ply(args.output, points, colors_rgb)

    print(f"Saved {args.output}")
    print(f"Points: {len(points)}")
    print(f"Depth resolution: {w}x{h}")
    print("K:")
    print(K)


if __name__ == "__main__":
    main()