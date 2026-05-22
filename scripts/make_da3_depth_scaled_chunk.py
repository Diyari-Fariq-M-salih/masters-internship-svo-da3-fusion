#!/usr/bin/env python3
"""
Build a DA3-inverted chunk after applying source->target depth scale corrections.

This is a diagnostic for overlapping chunks where the same RGB frames have
different DA3 depth scale depending on chunk context.
"""

import argparse
import json
from pathlib import Path

import cv2
import numpy as np


def da3_extrinsics_to_matrix(E):
    T = np.eye(4, dtype=np.float64)
    T[:3, :4] = E.astype(np.float64)
    return T


def da3_inv_pose(E):
    return np.linalg.inv(da3_extrinsics_to_matrix(E))


def camera_points_for_same_pixels(target_depth, target_K, source_depth, source_K, max_depth):
    if target_depth.shape != source_depth.shape:
        raise ValueError(
            f"Depth shape mismatch: target {target_depth.shape}, "
            f"source {source_depth.shape}"
        )

    h, w = target_depth.shape
    us, vs = np.meshgrid(np.arange(w), np.arange(h))

    z_t = target_depth.astype(np.float64)
    z_s = source_depth.astype(np.float64)
    valid = np.isfinite(z_t) & np.isfinite(z_s) & (z_t > 0) & (z_s > 0)
    if max_depth is not None:
        valid &= (z_t < max_depth) & (z_s < max_depth)

    us = us[valid].astype(np.float64)
    vs = vs[valid].astype(np.float64)
    z_t = z_t[valid]
    z_s = z_s[valid]

    def points(z, K):
        fx = float(K[0, 0])
        fy = float(K[1, 1])
        cx = float(K[0, 2])
        cy = float(K[1, 2])
        x = (us - cx) * z / fx
        y = (vs - cy) * z / fy
        return np.stack([x, y, z], axis=1)

    return points(z_t, target_K), points(z_s, source_K), z_t, z_s


def optimal_scale(source_points, target_points):
    denom = float(np.sum(source_points * source_points))
    if denom <= 0:
        raise ValueError("Cannot estimate scale from zero source points.")
    return float(np.sum(target_points * source_points) / denom)


def estimate_overlap_depth_scales(
    source_da3,
    target_da3,
    source_start_frame,
    target_start_frame,
    overlap_start_frame,
    overlap_end_frame,
    max_depth,
):
    scales = {}
    details = {}

    for frame_id in range(overlap_start_frame, overlap_end_frame + 1):
        source_idx = frame_id - source_start_frame
        target_idx = frame_id - target_start_frame
        if source_idx < 0 or source_idx >= source_da3["depth"].shape[0]:
            raise IndexError(f"Frame {frame_id} maps outside source DA3 chunk.")
        if target_idx < 0 or target_idx >= target_da3["depth"].shape[0]:
            raise IndexError(f"Frame {frame_id} maps outside target DA3 chunk.")

        target_points, source_points, target_z, source_z = camera_points_for_same_pixels(
            target_da3["depth"][target_idx],
            target_da3["intrinsics"][target_idx],
            source_da3["depth"][source_idx],
            source_da3["intrinsics"][source_idx],
            max_depth,
        )
        scale = optimal_scale(source_points, target_points)
        ratio = source_z / target_z
        residual = np.linalg.norm(scale * source_points - target_points, axis=1)

        scales[frame_id] = scale
        details[str(frame_id)] = {
            "source_idx": int(source_idx),
            "target_idx": int(target_idx),
            "source_to_target_depth_scale": float(scale),
            "source_over_target_depth_ratio_median": float(np.median(ratio)),
            "source_over_target_depth_ratio_mean": float(np.mean(ratio)),
            "scaled_camera_space_rmse": float(np.sqrt(np.mean(residual * residual))),
            "scaled_camera_space_median": float(np.median(residual)),
            "valid_pixel_count": int(source_points.shape[0]),
        }

    return scales, details


def scale_for_frame(frame_id, overlap_scales, overlap_start, overlap_end, mode):
    if mode == "median":
        return float(np.median(list(overlap_scales.values())))

    if mode != "per_frame_nearest":
        raise ValueError(f"Unknown scale mode: {mode}")

    if frame_id in overlap_scales:
        return float(overlap_scales[frame_id])

    if frame_id < overlap_start:
        return float(overlap_scales[overlap_start])
    if frame_id > overlap_end:
        return float(overlap_scales[overlap_end])

    return float(np.median(list(overlap_scales.values())))


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

    z = depth[vs, us].astype(np.float64)
    valid = np.isfinite(z) & (z > 0)
    if max_depth is not None:
        valid &= z < max_depth

    us = us[valid].astype(np.float64)
    vs = vs[valid].astype(np.float64)
    z = z[valid]

    x = (us - cx) * z / fx
    y = (vs - cy) * z / fy
    pts_c = np.stack([x, y, z], axis=1)
    pts_c_h = np.concatenate([pts_c, np.ones((pts_c.shape[0], 1))], axis=1)
    pts_w = (T_w_c @ pts_c_h.T).T[:, :3]

    colors_bgr = rgb_bgr[vs.astype(np.int64), us.astype(np.int64)]
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
        description="Build a source DA3-inv chunk with overlap-derived depth scale corrections."
    )
    parser.add_argument("--source_da3_npz", required=True)
    parser.add_argument("--target_da3_npz", required=True)
    parser.add_argument("--source_start_frame", type=int, required=True)
    parser.add_argument("--source_end_frame", type=int, required=True)
    parser.add_argument("--target_start_frame", type=int, required=True)
    parser.add_argument("--overlap_start_frame", type=int, required=True)
    parser.add_argument("--overlap_end_frame", type=int, required=True)
    parser.add_argument("--rgb_dir", required=True)
    parser.add_argument("--output", required=True)
    parser.add_argument("--report_json", default=None)
    parser.add_argument(
        "--scale_mode",
        choices=["per_frame_nearest", "median"],
        default="per_frame_nearest",
        help=(
            "per_frame_nearest uses measured overlap scales and clamps "
            "non-overlap frames to the nearest overlap scale. median applies "
            "the median overlap scale to the whole source chunk."
        ),
    )
    parser.add_argument("--stride", type=int, default=6)
    parser.add_argument("--max_depth", type=float, default=10.0)
    args = parser.parse_args()

    source_da3 = np.load(args.source_da3_npz)
    target_da3 = np.load(args.target_da3_npz)
    rgb_dir = Path(args.rgb_dir)

    overlap_scales, scale_details = estimate_overlap_depth_scales(
        source_da3,
        target_da3,
        args.source_start_frame,
        args.target_start_frame,
        args.overlap_start_frame,
        args.overlap_end_frame,
        args.max_depth,
    )

    all_points = []
    all_colors = []
    applied_scales = {}
    point_counts = {}

    for frame_id in range(args.source_start_frame, args.source_end_frame + 1):
        depth_idx = frame_id - args.source_start_frame
        if depth_idx < 0 or depth_idx >= source_da3["depth"].shape[0]:
            raise IndexError(f"Frame {frame_id} maps outside source DA3 chunk.")

        scale = scale_for_frame(
            frame_id,
            overlap_scales,
            args.overlap_start_frame,
            args.overlap_end_frame,
            args.scale_mode,
        )
        depth = source_da3["depth"][depth_idx].astype(np.float64) * scale
        K = source_da3["intrinsics"][depth_idx]
        T_w_c = da3_inv_pose(source_da3["extrinsics"][depth_idx])

        rgb_path = rgb_dir / f"frame_{frame_id:06d}.png"
        rgb = cv2.imread(str(rgb_path), cv2.IMREAD_COLOR)
        if rgb is None:
            raise FileNotFoundError(rgb_path)

        points, colors = backproject_frame(
            rgb,
            depth,
            K,
            T_w_c,
            stride=args.stride,
            max_depth=args.max_depth,
        )
        all_points.append(points)
        all_colors.append(colors)
        applied_scales[str(frame_id)] = float(scale)
        point_counts[str(frame_id)] = int(points.shape[0])

        print(
            f"Frame {frame_id} depth_idx {depth_idx} "
            f"scale={scale:.9f}: {points.shape[0]} points"
        )

    points = np.concatenate(all_points, axis=0)
    colors = np.concatenate(all_colors, axis=0)
    save_ply(args.output, points, colors)

    report = {
        "source_da3_npz": args.source_da3_npz,
        "target_da3_npz": args.target_da3_npz,
        "source_start_frame": args.source_start_frame,
        "source_end_frame": args.source_end_frame,
        "target_start_frame": args.target_start_frame,
        "overlap_start_frame": args.overlap_start_frame,
        "overlap_end_frame": args.overlap_end_frame,
        "scale_mode": args.scale_mode,
        "stride": args.stride,
        "max_depth": args.max_depth,
        "overlap_scale_details": scale_details,
        "applied_scales": applied_scales,
        "point_counts": point_counts,
        "point_count_total": int(points.shape[0]),
        "output": args.output,
    }
    if args.report_json is not None:
        report_path = Path(args.report_json)
        report_path.parent.mkdir(parents=True, exist_ok=True)
        with report_path.open("w") as f:
            json.dump(report, f, indent=2)
            f.write("\n")

    print("Built depth-scaled DA3-inv source chunk")
    print(f"  scale_mode: {args.scale_mode}")
    print(f"  overlap scales: {[round(overlap_scales[f], 9) for f in sorted(overlap_scales)]}")
    print(f"  output: {args.output}")
    print(f"  total points: {points.shape[0]}")
    if args.report_json is not None:
        print(f"  report: {args.report_json}")


if __name__ == "__main__":
    main()
