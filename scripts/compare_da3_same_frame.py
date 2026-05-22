#!/usr/bin/env python3
"""
Compare the same RGB frame as reconstructed by two different DA3 chunks.

This tests whether overlapping chunks are related by a clean global transform,
or whether DA3 predicts context-dependent depth/geometry for the same image.
"""

import argparse
import json
from pathlib import Path

import cv2
import numpy as np
from scipy.spatial import cKDTree


def da3_extrinsics_to_matrix(E):
    T = np.eye(4, dtype=np.float64)
    T[:3, :4] = E.astype(np.float64)
    return T


def da3_inv_pose(E):
    return np.linalg.inv(da3_extrinsics_to_matrix(E))


def load_frame(da3_npz, chunk_start, frame_id):
    da3 = np.load(da3_npz)
    depth_idx = frame_id - chunk_start
    if depth_idx < 0 or depth_idx >= da3["depth"].shape[0]:
        raise IndexError(
            f"Frame {frame_id} maps to DA3 index {depth_idx}, "
            f"but {da3_npz} depth has shape {da3['depth'].shape}."
        )

    return {
        "depth_idx": depth_idx,
        "depth": da3["depth"][depth_idx].astype(np.float64),
        "K": da3["intrinsics"][depth_idx].astype(np.float64),
        "T_w_c": da3_inv_pose(da3["extrinsics"][depth_idx]),
    }


def load_rgb(rgb_dir, frame_id, size):
    rgb_path = Path(rgb_dir) / f"frame_{frame_id:06d}.png"
    rgb = cv2.imread(str(rgb_path), cv2.IMREAD_COLOR)
    if rgb is None:
        raise FileNotFoundError(rgb_path)
    rgb = cv2.resize(rgb, size, interpolation=cv2.INTER_AREA)
    return rgb[:, :, ::-1]


def backproject(depth, K, T_w_c, stride, max_depth):
    h, w = depth.shape
    fx = float(K[0, 0])
    fy = float(K[1, 1])
    cx = float(K[0, 2])
    cy = float(K[1, 2])

    us, vs = np.meshgrid(
        np.arange(0, w, stride),
        np.arange(0, h, stride),
    )

    z = depth[vs, us]
    valid = np.isfinite(z) & (z > 0)
    if max_depth is not None:
        valid &= z < max_depth

    us = us[valid].astype(np.float64)
    vs = vs[valid].astype(np.float64)
    z = z[valid].astype(np.float64)

    x = (us - cx) * z / fx
    y = (vs - cy) * z / fy
    pts_c = np.stack([x, y, z], axis=1)

    pts_c_h = np.concatenate([pts_c, np.ones((pts_c.shape[0], 1))], axis=1)
    pts_w = (T_w_c @ pts_c_h.T).T[:, :3]

    pixel_indices = np.stack([vs.astype(np.int64), us.astype(np.int64)], axis=1)
    return pts_c, pts_w, pixel_indices


def camera_points_for_same_pixels(target, source, max_depth):
    if target["depth"].shape != source["depth"].shape:
        raise ValueError(
            f"Depth shape mismatch: target {target['depth'].shape}, "
            f"source {source['depth'].shape}"
        )

    h, w = target["depth"].shape
    us, vs = np.meshgrid(np.arange(w), np.arange(h))
    z_t = target["depth"]
    z_s = source["depth"]

    valid = np.isfinite(z_t) & np.isfinite(z_s) & (z_t > 0) & (z_s > 0)
    if max_depth is not None:
        valid &= (z_t < max_depth) & (z_s < max_depth)

    us = us[valid].astype(np.float64)
    vs = vs[valid].astype(np.float64)
    z_t = z_t[valid].astype(np.float64)
    z_s = z_s[valid].astype(np.float64)

    def pts_from_depth(z, K):
        fx = float(K[0, 0])
        fy = float(K[1, 1])
        cx = float(K[0, 2])
        cy = float(K[1, 2])
        x = (us - cx) * z / fx
        y = (vs - cy) * z / fy
        return np.stack([x, y, z], axis=1)

    return pts_from_depth(z_t, target["K"]), pts_from_depth(z_s, source["K"]), z_t, z_s


def optimal_scale(source_points, target_points):
    denom = float(np.sum(source_points * source_points))
    if denom <= 0:
        raise ValueError("Cannot estimate scale from zero source points.")
    return float(np.sum(target_points * source_points) / denom)


def stats(values):
    values = np.asarray(values, dtype=np.float64)
    return {
        "rmse": float(np.sqrt(np.mean(values * values))),
        "median": float(np.median(values)),
        "mean": float(np.mean(values)),
        "p90": float(np.percentile(values, 90)),
        "p95": float(np.percentile(values, 95)),
        "max": float(np.max(values)),
    }


def ratio_stats(ratio):
    ratio = np.asarray(ratio, dtype=np.float64)
    return {
        "median": float(np.median(ratio)),
        "mean": float(np.mean(ratio)),
        "p05": float(np.percentile(ratio, 5)),
        "p25": float(np.percentile(ratio, 25)),
        "p75": float(np.percentile(ratio, 75)),
        "p95": float(np.percentile(ratio, 95)),
        "std": float(np.std(ratio)),
    }


def nearest_neighbor_stats(source_points, target_points):
    tree = cKDTree(target_points)
    distances, _ = tree.query(source_points, k=1, workers=-1)
    return stats(distances)


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


def transform_pose_only(source_world, source_T_w_c, target_T_w_c):
    T_target_source = target_T_w_c @ np.linalg.inv(source_T_w_c)
    pts_h = np.concatenate([source_world, np.ones((source_world.shape[0], 1))], axis=1)
    return (T_target_source @ pts_h.T).T[:, :3]


def transform_pose_with_camera_scale(source_world, source_T_w_c, target_T_w_c, scale):
    R_t = target_T_w_c[:3, :3]
    t_t = target_T_w_c[:3, 3]
    R_s = source_T_w_c[:3, :3]
    t_s = source_T_w_c[:3, 3]

    rotation = R_t @ R_s.T
    translation = t_t - scale * rotation @ t_s
    return scale * (rotation @ source_world.T).T + translation


def main():
    parser = argparse.ArgumentParser(
        description="Compare one overlapping frame from two DA3 chunks."
    )
    parser.add_argument("--target_da3_npz", required=True)
    parser.add_argument("--source_da3_npz", required=True)
    parser.add_argument("--target_start_frame", type=int, required=True)
    parser.add_argument("--source_start_frame", type=int, required=True)
    parser.add_argument("--frame_id", type=int, required=True)
    parser.add_argument("--rgb_dir", required=True)
    parser.add_argument("--output_dir", required=True)
    parser.add_argument("--label", default=None)
    parser.add_argument("--stride", type=int, default=6)
    parser.add_argument("--max_depth", type=float, default=10.0)
    args = parser.parse_args()

    label = args.label or f"frame_{args.frame_id:06d}"
    output_dir = Path(args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)

    target = load_frame(args.target_da3_npz, args.target_start_frame, args.frame_id)
    source = load_frame(args.source_da3_npz, args.source_start_frame, args.frame_id)

    target_pts_c_full, source_pts_c_full, z_t, z_s = camera_points_for_same_pixels(
        target,
        source,
        args.max_depth,
    )
    source_to_target_depth_scale = optimal_scale(source_pts_c_full, target_pts_c_full)
    camera_scaled_residuals = np.linalg.norm(
        source_to_target_depth_scale * source_pts_c_full - target_pts_c_full,
        axis=1,
    )

    target_pts_c, target_pts_w, target_pixels = backproject(
        target["depth"],
        target["K"],
        target["T_w_c"],
        stride=args.stride,
        max_depth=args.max_depth,
    )
    source_pts_c, source_pts_w, source_pixels = backproject(
        source["depth"],
        source["K"],
        source["T_w_c"],
        stride=args.stride,
        max_depth=args.max_depth,
    )

    size = (target["depth"].shape[1], target["depth"].shape[0])
    rgb = load_rgb(args.rgb_dir, args.frame_id, size)
    target_colors = rgb[target_pixels[:, 0], target_pixels[:, 1]]
    source_colors = rgb[source_pixels[:, 0], source_pixels[:, 1]]

    source_pose_aligned = transform_pose_only(
        source_pts_w,
        source["T_w_c"],
        target["T_w_c"],
    )
    source_pose_scale_aligned = transform_pose_with_camera_scale(
        source_pts_w,
        source["T_w_c"],
        target["T_w_c"],
        source_to_target_depth_scale,
    )

    target_path = output_dir / f"{label}_target_frame.ply"
    source_path = output_dir / f"{label}_source_frame.ply"
    pose_aligned_path = output_dir / f"{label}_source_pose_aligned_to_target.ply"
    pose_scale_path = output_dir / f"{label}_source_pose_depthscale_aligned_to_target.ply"
    merged_pose_path = output_dir / f"{label}_pose_aligned_merged_colored.ply"
    merged_pose_scale_path = output_dir / f"{label}_pose_depthscale_aligned_merged_colored.ply"
    report_path = output_dir / f"{label}_report.json"

    save_ascii_ply(target_path, target_pts_w, target_colors)
    save_ascii_ply(source_path, source_pts_w, source_colors)
    save_ascii_ply(pose_aligned_path, source_pose_aligned, source_colors)
    save_ascii_ply(pose_scale_path, source_pose_scale_aligned, source_colors)

    target_diag_colors = np.tile(np.array([[80, 180, 255]], dtype=np.uint8), (target_pts_w.shape[0], 1))
    source_diag_colors = np.tile(np.array([[255, 120, 80]], dtype=np.uint8), (source_pts_w.shape[0], 1))

    save_ascii_ply(
        merged_pose_path,
        np.concatenate([target_pts_w, source_pose_aligned], axis=0),
        np.concatenate([target_diag_colors, source_diag_colors], axis=0),
    )
    save_ascii_ply(
        merged_pose_scale_path,
        np.concatenate([target_pts_w, source_pose_scale_aligned], axis=0),
        np.concatenate([target_diag_colors, source_diag_colors], axis=0),
    )

    depth_ratio = z_s / z_t
    direct_camera_residuals = np.linalg.norm(source_pts_c_full - target_pts_c_full, axis=1)

    report = {
        "frame_id": args.frame_id,
        "target_da3_npz": args.target_da3_npz,
        "source_da3_npz": args.source_da3_npz,
        "target_start_frame": args.target_start_frame,
        "source_start_frame": args.source_start_frame,
        "target_depth_idx": target["depth_idx"],
        "source_depth_idx": source["depth_idx"],
        "depth_shape": list(target["depth"].shape),
        "stride": args.stride,
        "max_depth": args.max_depth,
        "intrinsics_max_abs_diff": float(np.max(np.abs(target["K"] - source["K"]))),
        "depth_source_over_target_ratio": ratio_stats(depth_ratio),
        "source_to_target_camera_depth_scale": source_to_target_depth_scale,
        "camera_space_same_pixel_error_no_scale": stats(direct_camera_residuals),
        "camera_space_same_pixel_error_with_depth_scale": stats(camera_scaled_residuals),
        "target_point_count": int(target_pts_w.shape[0]),
        "source_point_count": int(source_pts_w.shape[0]),
        "nn_error_source_pose_aligned_to_target": nearest_neighbor_stats(
            source_pose_aligned,
            target_pts_w,
        ),
        "nn_error_source_pose_depthscale_aligned_to_target": nearest_neighbor_stats(
            source_pose_scale_aligned,
            target_pts_w,
        ),
        "outputs": {
            "target_frame": str(target_path),
            "source_frame": str(source_path),
            "source_pose_aligned_to_target": str(pose_aligned_path),
            "source_pose_depthscale_aligned_to_target": str(pose_scale_path),
            "pose_aligned_merged_colored": str(merged_pose_path),
            "pose_depthscale_aligned_merged_colored": str(merged_pose_scale_path),
        },
    }

    with report_path.open("w") as f:
        json.dump(report, f, indent=2)
        f.write("\n")

    print(f"Compared DA3 same-frame reconstruction for frame {args.frame_id}")
    print(f"  target depth_idx: {target['depth_idx']}")
    print(f"  source depth_idx: {source['depth_idx']}")
    print(f"  source/target depth ratio median: {np.median(depth_ratio):.9f}")
    print(f"  source -> target depth scale: {source_to_target_depth_scale:.9f}")
    print(
        "  camera-space same-pixel error with scale: "
        f"rmse={np.sqrt(np.mean(camera_scaled_residuals * camera_scaled_residuals)):.9f}, "
        f"median={np.median(camera_scaled_residuals):.9f}, "
        f"p95={np.percentile(camera_scaled_residuals, 95):.9f}"
    )
    pose_nn = nearest_neighbor_stats(source_pose_aligned, target_pts_w)
    pose_scale_nn = nearest_neighbor_stats(source_pose_scale_aligned, target_pts_w)
    print(
        "  pose-only merged NN error: "
        f"rmse={pose_nn['rmse']:.9f}, median={pose_nn['median']:.9f}, p95={pose_nn['p95']:.9f}"
    )
    print(
        "  pose+depth-scale merged NN error: "
        f"rmse={pose_scale_nn['rmse']:.9f}, median={pose_scale_nn['median']:.9f}, "
        f"p95={pose_scale_nn['p95']:.9f}"
    )
    print(f"Saved report: {report_path}")
    print(f"Open: {merged_pose_path}")
    print(f"Open: {merged_pose_scale_path}")


if __name__ == "__main__":
    main()
