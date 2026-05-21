#!/usr/bin/env python3
"""
Align a finished DA3-inverted reconstruction chunk into the SVO frame.

Preferred workflow:
1. Build the local chunk with make_pcl_chunk_from_da3_npz.py --pose_source da3_inv.
2. Use this script to estimate DA3-inverted camera centers -> SVO centers.
3. Apply that chunk-level Sim(3) to the whole PLY.

This preserves DA3's depth/pose consistency while anchoring the chunk to SVO.
"""

import argparse
import csv
from pathlib import Path

import numpy as np


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


def da3_extrinsics_to_matrix(E):
    T = np.eye(4, dtype=np.float64)
    T[:3, :4] = E.astype(np.float64)
    return T


def estimate_sim3_umeyama(source, target):
    """
    Estimate target ~= scale * rotation @ source + translation.
    """
    if source.shape != target.shape:
        raise ValueError(f"Shape mismatch: source {source.shape}, target {target.shape}")
    if source.ndim != 2 or source.shape[1] != 3:
        raise ValueError(f"Expected Nx3 point arrays, got {source.shape}")
    if source.shape[0] < 3:
        raise ValueError("At least 3 matched centers are required for Sim(3).")

    mu_source = source.mean(axis=0)
    mu_target = target.mean(axis=0)
    X = source - mu_source
    Y = target - mu_target

    cov = (Y.T @ X) / source.shape[0]
    U, singular_values, Vt = np.linalg.svd(cov)

    sign = np.eye(3)
    if np.linalg.det(U @ Vt) < 0:
        sign[-1, -1] = -1.0

    rotation = U @ sign @ Vt
    source_variance = np.mean(np.sum(X * X, axis=1))
    if source_variance <= 0:
        raise ValueError("Source centers have zero variance; cannot estimate scale.")

    scale = float(np.trace(np.diag(singular_values) @ sign) / source_variance)
    translation = mu_target - scale * rotation @ mu_source

    aligned = scale * (rotation @ source.T).T + translation
    errors = np.linalg.norm(aligned - target, axis=1)

    return scale, rotation, translation, errors


def load_centers(sync_csv, da3_npz, start_frame, end_frame, max_dt):
    rows = load_sync_rows(sync_csv, start_frame, end_frame, max_dt)
    if not rows:
        raise RuntimeError(
            "No synchronized rows found. Check start_frame, end_frame, and max_dt."
        )

    da3 = np.load(da3_npz)
    if "extrinsics" not in da3:
        raise KeyError("DA3 npz does not contain 'extrinsics'.")

    E_all = da3["extrinsics"]
    svo_centers = []
    da3_inv_centers = []

    for row in rows:
        frame_id = int(row["frame_id"])
        depth_idx = frame_id - start_frame
        if depth_idx < 0 or depth_idx >= E_all.shape[0]:
            raise IndexError(
                f"Frame {frame_id} maps to DA3 index {depth_idx}, "
                f"but DA3 extrinsics have shape {E_all.shape}."
            )

        svo_centers.append([float(row["tx"]), float(row["ty"]), float(row["tz"])])

        T_da3 = da3_extrinsics_to_matrix(E_all[depth_idx])
        T_da3_inv = np.linalg.inv(T_da3)
        da3_inv_centers.append(T_da3_inv[:3, 3])

    return np.array(da3_inv_centers), np.array(svo_centers)


def load_ascii_ply(path):
    path = Path(path)
    header = []
    vertex_count = None

    with path.open("r") as f:
        for line in f:
            header.append(line)
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
            if len(vals) < 6:
                raise ValueError(f"Expected at least x y z r g b, got: {line!r}")
            rows.append(vals)

    points = np.array([[float(v[0]), float(v[1]), float(v[2])] for v in rows])
    colors = np.array([[int(v[3]), int(v[4]), int(v[5])] for v in rows], dtype=np.uint8)
    return header, points, colors


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
        description=(
            "Transform a DA3-inverted reconstruction chunk into the SVO trajectory "
            "frame using a Sim(3) estimated from matched camera centers."
        )
    )
    parser.add_argument("--input_ply", required=True)
    parser.add_argument("--output_ply", required=True)
    parser.add_argument("--sync_csv", required=True)
    parser.add_argument("--da3_npz", required=True)
    parser.add_argument("--start_frame", type=int, required=True)
    parser.add_argument("--end_frame", type=int, required=True)
    parser.add_argument("--max_dt", type=float, default=0.20)
    args = parser.parse_args()

    da3_centers, svo_centers = load_centers(
        sync_csv=args.sync_csv,
        da3_npz=args.da3_npz,
        start_frame=args.start_frame,
        end_frame=args.end_frame,
        max_dt=args.max_dt,
    )
    scale, rotation, translation, errors = estimate_sim3_umeyama(
        source=da3_centers,
        target=svo_centers,
    )

    _, points, colors = load_ascii_ply(args.input_ply)
    aligned_points = scale * (rotation @ points.T).T + translation
    save_ascii_ply(args.output_ply, aligned_points, colors)

    print("Estimated Sim(3), DA3-inverted -> SVO:")
    print(f"  scale: {scale:.9f}")
    print("  rotation:")
    for r in rotation:
        print(f"    {r[0]: .9f} {r[1]: .9f} {r[2]: .9f}")
    print(
        "  translation: "
        f"{translation[0]: .9f} {translation[1]: .9f} {translation[2]: .9f}"
    )
    print(
        "  center alignment error: "
        f"rmse={np.sqrt(np.mean(errors * errors)):.9f}, "
        f"median={np.median(errors):.9f}, "
        f"max={np.max(errors):.9f}"
    )
    print(f"Transformed points: {points.shape[0]}")
    print(f"Saved: {args.output_ply}")


if __name__ == "__main__":
    main()
