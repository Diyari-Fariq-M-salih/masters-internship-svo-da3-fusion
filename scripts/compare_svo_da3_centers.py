#!/usr/bin/env python3

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

    frame_ids = []
    pose_indices = []
    dts = []
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

        T_da3 = da3_extrinsics_to_matrix(E_all[depth_idx])
        T_da3_inv = np.linalg.inv(T_da3)

        frame_ids.append(frame_id)
        pose_indices.append(int(row["pose_index"]))
        dts.append(float(row["dt"]))
        svo_centers.append([float(row["tx"]), float(row["ty"]), float(row["tz"])])
        da3_inv_centers.append(T_da3_inv[:3, 3])

    return (
        np.array(frame_ids, dtype=np.int64),
        np.array(pose_indices, dtype=np.int64),
        np.array(dts, dtype=np.float64),
        np.array(svo_centers, dtype=np.float64),
        np.array(da3_inv_centers, dtype=np.float64),
    )


def path_length(points):
    if len(points) < 2:
        return 0.0
    steps = np.linalg.norm(np.diff(points, axis=0), axis=1)
    return float(steps.sum())


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

    return scale, rotation, translation, aligned, errors


def print_center_table(frame_ids, pose_indices, dts, svo_centers, da3_inv_centers):
    print("Matched centers:")
    print(
        "frame pose_idx dt "
        "svo_x svo_y svo_z "
        "da3inv_x da3inv_y da3inv_z "
        "center_delta"
    )
    for frame_id, pose_idx, dt, svo, da3 in zip(
        frame_ids, pose_indices, dts, svo_centers, da3_inv_centers
    ):
        delta = np.linalg.norm(svo - da3)
        print(
            f"{frame_id:5d} {pose_idx:8d} {dt:.6f} "
            f"{svo[0]: .6f} {svo[1]: .6f} {svo[2]: .6f} "
            f"{da3[0]: .6f} {da3[1]: .6f} {da3[2]: .6f} "
            f"{delta:.6f}"
        )


def main():
    parser = argparse.ArgumentParser(
        description=(
            "Compare SVO camera centers against DA3-inverted camera centers "
            "and estimate a Sim(3)-style alignment."
        )
    )
    parser.add_argument("--sync_csv", required=True)
    parser.add_argument("--da3_npz", required=True)
    parser.add_argument("--start_frame", type=int, required=True)
    parser.add_argument("--end_frame", type=int, required=True)
    parser.add_argument("--max_dt", type=float, default=0.20)
    parser.add_argument(
        "--print_centers",
        action="store_true",
        help="Print one row per matched frame before the summary.",
    )
    args = parser.parse_args()

    sync_csv = Path(args.sync_csv)
    da3_npz = Path(args.da3_npz)

    frame_ids, pose_indices, dts, svo_centers, da3_inv_centers = load_centers(
        sync_csv=sync_csv,
        da3_npz=da3_npz,
        start_frame=args.start_frame,
        end_frame=args.end_frame,
        max_dt=args.max_dt,
    )

    scale, rotation, translation, aligned, errors = estimate_sim3_umeyama(
        source=svo_centers,
        target=da3_inv_centers,
    )

    svo_path = path_length(svo_centers)
    da3_path = path_length(da3_inv_centers)
    aligned_path = path_length(aligned)

    if args.print_centers:
        print_center_table(
            frame_ids,
            pose_indices,
            dts,
            svo_centers,
            da3_inv_centers,
        )
        print()

    print("Inputs:")
    print(f"  sync_csv: {sync_csv}")
    print(f"  da3_npz:  {da3_npz}")
    print(f"  frames:   {int(frame_ids[0])}..{int(frame_ids[-1])}")
    print(f"  count:    {len(frame_ids)}")
    print(f"  max_dt:   {float(dts.max()):.9f}")
    print()

    print("Path lengths:")
    print(f"  SVO:              {svo_path:.9f}")
    print(f"  DA3 inverted:     {da3_path:.9f}")
    print(f"  aligned SVO:      {aligned_path:.9f}")
    if svo_path > 0:
        print(f"  DA3/SVO ratio:    {da3_path / svo_path:.9f}")
    print()

    print("Sim(3) estimate, SVO -> DA3-inverted:")
    print(f"  scale:       {scale:.9f}")
    print("  rotation:")
    for row in rotation:
        print(f"    {row[0]: .9f} {row[1]: .9f} {row[2]: .9f}")
    print(
        "  translation: "
        f"{translation[0]: .9f} {translation[1]: .9f} {translation[2]: .9f}"
    )
    print()

    print("Alignment error:")
    print(f"  rmse:     {np.sqrt(np.mean(errors * errors)):.9f}")
    print(f"  median:   {np.median(errors):.9f}")
    print(f"  mean:     {np.mean(errors):.9f}")
    print(f"  max:      {np.max(errors):.9f}")


if __name__ == "__main__":
    main()
