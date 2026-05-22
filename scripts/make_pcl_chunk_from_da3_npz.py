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


def da3_extrinsics_to_matrix(E):
    """
    Convert DA3 3x4 extrinsics to a 4x4 transform.

    For this diagnostic, we use DA3 extrinsics directly as a T_w_c-like
    transform. If the resulting cloud looks inverted or odd, we can later
    test the inverse convention as a separate option.
    """
    T = np.eye(4, dtype=np.float32)
    T[:3, :4] = E.astype(np.float32)
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


def estimate_svo_to_da3_inv_sim3(rows, E_all, start_frame):
    if E_all is None:
        raise KeyError("DA3 npz does not contain 'extrinsics'.")

    svo_centers = []
    da3_inv_centers = []

    for row in rows:
        frame_id = int(row["frame_id"])
        depth_idx = frame_id - start_frame
        if depth_idx < 0 or depth_idx >= E_all.shape[0]:
            raise IndexError(
                f"Frame {frame_id} maps to DA3 depth index {depth_idx}, "
                f"but extrinsics array has shape {E_all.shape}."
            )

        svo_centers.append([float(row["tx"]), float(row["ty"]), float(row["tz"])])

        T_da3 = da3_extrinsics_to_matrix(E_all[depth_idx])
        T_da3_inv = np.linalg.inv(T_da3)
        da3_inv_centers.append(T_da3_inv[:3, 3])

    scale, rotation, translation, errors = estimate_sim3_umeyama(
        np.array(svo_centers, dtype=np.float64),
        np.array(da3_inv_centers, dtype=np.float64),
    )

    T_sim3 = np.eye(4, dtype=np.float32)
    T_sim3[:3, :3] = (scale * rotation).astype(np.float32)
    T_sim3[:3, 3] = translation.astype(np.float32)

    return T_sim3, scale, rotation, translation, errors


def estimate_da3_inv_to_svo_sim3(rows, E_all, da3_start_frame):
    if E_all is None:
        raise KeyError("DA3 npz does not contain 'extrinsics'.")

    da3_inv_centers = []
    svo_centers = []

    for row in rows:
        frame_id = int(row["frame_id"])
        depth_idx = frame_id - da3_start_frame
        if depth_idx < 0 or depth_idx >= E_all.shape[0]:
            raise IndexError(
                f"Frame {frame_id} maps to DA3 depth index {depth_idx}, "
                f"but extrinsics array has shape {E_all.shape}."
            )

        T_da3 = da3_extrinsics_to_matrix(E_all[depth_idx])
        T_da3_inv = np.linalg.inv(T_da3)
        da3_inv_centers.append(T_da3_inv[:3, 3])
        svo_centers.append([float(row["tx"]), float(row["ty"]), float(row["tz"])])

    scale, rotation, translation, errors = estimate_sim3_umeyama(
        np.array(da3_inv_centers, dtype=np.float64),
        np.array(svo_centers, dtype=np.float64),
    )

    return scale, rotation, translation, errors


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


def choose_pose_matrix(args, row, E_all, depth_idx, T_svo_to_da3_inv):
    if args.pose_source == "svo":
        return pose_to_matrix(row)

    if args.pose_source == "svo_sim3_da3inv":
        if T_svo_to_da3_inv is None:
            raise RuntimeError("SVO to DA3-inverted Sim(3) was not estimated.")
        # Diagnostic mode only: visually worse than DA3-inv chunk alignment so far.
        return (T_svo_to_da3_inv @ pose_to_matrix(row)).astype(np.float32)

    if args.pose_source == "da3":
        if E_all is None:
            raise KeyError("DA3 npz does not contain 'extrinsics'.")
        return da3_extrinsics_to_matrix(E_all[depth_idx])

    if args.pose_source == "da3_inv":
        if E_all is None:
            raise KeyError("DA3 npz does not contain 'extrinsics'.")
        T = da3_extrinsics_to_matrix(E_all[depth_idx])
        return np.linalg.inv(T).astype(np.float32)

    if args.pose_source == "identity":
        return np.eye(4, dtype=np.float32)

    raise ValueError(f"Unknown pose_source: {args.pose_source}")


def main():
    parser = argparse.ArgumentParser(
        description="Fuse DA3 depth chunk with selected poses into one colored PLY."
    )
    parser.add_argument("--rgb_dir", required=True)
    parser.add_argument("--da3_npz", required=True)
    parser.add_argument("--sync_csv", required=True)
    parser.add_argument("--output", required=True)
    parser.add_argument("--start_frame", type=int, default=1)
    parser.add_argument("--end_frame", type=int, default=15)
    parser.add_argument(
        "--da3_start_frame",
        type=int,
        default=None,
        help=(
            "Original frame id corresponding to DA3 depth index 0. Defaults to "
            "--start_frame for backwards compatibility."
        ),
    )
    parser.add_argument("--stride", type=int, default=6)
    parser.add_argument("--max_dt", type=float, default=0.20)
    parser.add_argument("--max_depth", type=float, default=None)
    parser.add_argument(
        "--depth_scale",
        type=float,
        default=1.0,
        help="Scalar applied to DA3 depth values before backprojection.",
    )
    parser.add_argument(
        "--auto_depth_scale_from_da3_inv_to_svo",
        action="store_true",
        help=(
            "Estimate DA3-inv -> SVO Sim(3) from camera centers and multiply "
            "depths by the estimated scale. Intended for trajectory-first "
            "fusion where SVO poses place DA3 depth maps in the global frame."
        ),
    )
    parser.add_argument(
        "--scale_start_frame",
        type=int,
        default=None,
        help="First frame used for automatic depth-scale estimation.",
    )
    parser.add_argument(
        "--scale_end_frame",
        type=int,
        default=None,
        help="Last frame used for automatic depth-scale estimation.",
    )
    parser.add_argument(
        "--pose_source",
        choices=["svo", "svo_sim3_da3inv", "da3", "da3_inv", "identity"],
        default="svo",
        help=(
            "Pose source used to place each DA3 depth map into the output cloud. "
            "svo_sim3_da3inv is experimental; the preferred pipeline is "
            "da3_inv followed by align_da3inv_ply_to_svo.py."
        ),
    )
    args = parser.parse_args()

    rgb_dir = Path(args.rgb_dir)
    da3 = np.load(args.da3_npz)
    da3_start_frame = (
        args.da3_start_frame if args.da3_start_frame is not None else args.start_frame
    )

    depth_all = da3["depth"]
    K_all = da3["intrinsics"]
    E_all = da3["extrinsics"] if "extrinsics" in da3 else None

    rows = load_sync_rows(
        args.sync_csv,
        start_frame=args.start_frame,
        end_frame=args.end_frame,
        max_dt=args.max_dt,
    )

    if not rows:
        raise RuntimeError(
            "No synchronized rows found. Check start_frame, end_frame, and max_dt."
        )

    print(f"Using {len(rows)} synchronized frames")
    print(f"Pose source: {args.pose_source}")
    print(f"DA3 depth shape: {depth_all.shape}")
    print(f"DA3 intrinsics shape: {K_all.shape}")
    if E_all is not None:
        print(f"DA3 extrinsics shape: {E_all.shape}")

    T_svo_to_da3_inv = None
    if args.pose_source == "svo_sim3_da3inv":
        (
            T_svo_to_da3_inv,
            sim3_scale,
            sim3_rotation,
            sim3_translation,
            sim3_errors,
        ) = estimate_svo_to_da3_inv_sim3(rows, E_all, da3_start_frame)

        print("Estimated Sim(3), SVO -> DA3-inverted:")
        print(f"  scale: {sim3_scale:.9f}")
        print("  rotation:")
        for r in sim3_rotation:
            print(f"    {r[0]: .9f} {r[1]: .9f} {r[2]: .9f}")
        print(
            "  translation: "
            f"{sim3_translation[0]: .9f} "
            f"{sim3_translation[1]: .9f} "
            f"{sim3_translation[2]: .9f}"
        )
        print(
            "  center alignment error: "
            f"rmse={np.sqrt(np.mean(sim3_errors * sim3_errors)):.9f}, "
            f"median={np.median(sim3_errors):.9f}, "
            f"max={np.max(sim3_errors):.9f}"
        )

    depth_scale = float(args.depth_scale)
    if args.auto_depth_scale_from_da3_inv_to_svo:
        scale_start_frame = (
            args.scale_start_frame
            if args.scale_start_frame is not None
            else args.start_frame
        )
        scale_end_frame = (
            args.scale_end_frame if args.scale_end_frame is not None else args.end_frame
        )
        scale_rows = load_sync_rows(
            args.sync_csv,
            start_frame=scale_start_frame,
            end_frame=scale_end_frame,
            max_dt=args.max_dt,
        )
        if not scale_rows:
            raise RuntimeError("No synchronized rows found for depth-scale estimation.")

        (
            auto_depth_scale,
            auto_scale_rotation,
            auto_scale_translation,
            auto_scale_errors,
        ) = estimate_da3_inv_to_svo_sim3(
            scale_rows,
            E_all,
            da3_start_frame,
        )
        depth_scale *= auto_depth_scale

        print("Estimated Sim(3), DA3-inverted -> SVO for depth scale:")
        print(f"  scale frames: {scale_start_frame}-{scale_end_frame}")
        print(f"  scale: {auto_depth_scale:.9f}")
        print(
            "  center alignment error: "
            f"rmse={np.sqrt(np.mean(auto_scale_errors * auto_scale_errors)):.9f}, "
            f"median={np.median(auto_scale_errors):.9f}, "
            f"max={np.max(auto_scale_errors):.9f}"
        )

    print(f"Depth scale: {depth_scale:.9f}")

    all_points = []
    all_colors = []

    for row in rows:
        frame_id = int(row["frame_id"])
        filename = row["filename"]
        rgb_path = rgb_dir / filename

        rgb = cv2.imread(str(rgb_path), cv2.IMREAD_COLOR)
        if rgb is None:
            raise FileNotFoundError(rgb_path)

        # DA3 chunk indexing:
        # original frame_id start_frame maps to DA3 depth index 0.
        depth_idx = frame_id - da3_start_frame

        if depth_idx < 0 or depth_idx >= depth_all.shape[0]:
            raise IndexError(
                f"Frame {frame_id} maps to DA3 depth index {depth_idx}, "
                f"but depth array has shape {depth_all.shape}. "
                f"Check start_frame/end_frame and DA3 chunk."
            )

        depth = depth_all[depth_idx].astype(np.float32) * depth_scale
        K = K_all[depth_idx]
        T_w_c = choose_pose_matrix(
            args,
            row,
            E_all,
            depth_idx,
            T_svo_to_da3_inv,
        )

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

        print(
            f"Frame {frame_id} depth_idx {depth_idx} "
            f"pose_source={args.pose_source}: {pts.shape[0]} points"
        )

    points = np.concatenate(all_points, axis=0)
    colors = np.concatenate(all_colors, axis=0)

    save_ply(args.output, points, colors)

    print(f"Saved: {args.output}")
    print(f"Total points: {points.shape[0]}")


if __name__ == "__main__":
    main()
