#!/usr/bin/env python3
"""
Merge generated ASCII PLY point clouds by concatenating vertices.

This script expects the simple PLY format written by the fusion scripts:
x y z red green blue, with one vertex per line.
"""

import argparse
from pathlib import Path

import numpy as np


def load_ascii_ply(path):
    path = Path(path)
    vertex_count = None

    with path.open("r") as f:
        for line in f:
            stripped = line.strip()
            if stripped.startswith("element vertex "):
                vertex_count = int(stripped.split()[-1])
            if stripped == "end_header":
                break

        if vertex_count is None:
            raise ValueError(f"PLY header does not contain vertex count: {path}")

        rows = []
        for _ in range(vertex_count):
            line = f.readline()
            if not line:
                raise ValueError(f"PLY ended before {vertex_count} vertices: {path}")
            vals = line.split()
            if len(vals) != 6:
                raise ValueError(
                    f"Expected x y z red green blue in {path}, got: {line!r}"
                )
            rows.append(vals)

    points = np.array([[float(v[0]), float(v[1]), float(v[2])] for v in rows])
    colors = np.array([[int(v[3]), int(v[4]), int(v[5])] for v in rows], dtype=np.uint8)
    return points, colors


def save_ascii_ply(path, points, colors):
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
        description="Merge simple generated ASCII PLY point clouds."
    )
    parser.add_argument("inputs", nargs="+", help="Input PLY files.")
    parser.add_argument("--output", required=True, help="Output merged PLY.")
    args = parser.parse_args()

    all_points = []
    all_colors = []

    for input_path in args.inputs:
        points, colors = load_ascii_ply(input_path)
        all_points.append(points)
        all_colors.append(colors)
        print(f"Loaded {input_path}: {points.shape[0]} points")

    merged_points = np.concatenate(all_points, axis=0)
    merged_colors = np.concatenate(all_colors, axis=0)

    save_ascii_ply(args.output, merged_points, merged_colors)

    print(f"Saved: {args.output}")
    print(f"Total points: {merged_points.shape[0]}")


if __name__ == "__main__":
    main()
