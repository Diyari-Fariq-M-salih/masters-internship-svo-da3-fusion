#!/usr/bin/env python3
"""
Calibrate DA3 depth against TUM RGB-D sensor depth and build GT-pose PLYs.

This is a proof-of-concept for the "anchor DA3 depth before fusion" path:
- TUM sensor depth provides metric per-frame depth anchors.
- DA3 depth is corrected per frame with several simple models.
- Corrected depths are backprojected with real camera intrinsics and placed
  with TUM ground-truth camera poses.

The same idea can later be ported to EuRoC by replacing sensor depth anchors
with sparse SVO/COLMAP triangulated depths.
"""

import argparse
import csv
import json
from pathlib import Path

import cv2
import numpy as np
from scipy.spatial.transform import Rotation as Rotation


def load_sync_rows(path, start_frame, end_frame):
    rows = []
    with Path(path).open("r") as f:
        reader = csv.DictReader(f)
        for row in reader:
            frame_id = int(row["frame_id"])
            if start_frame <= frame_id <= end_frame:
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

    T = np.eye(4, dtype=np.float64)
    T[:3, :3] = Rotation.from_quat([qx, qy, qz, qw]).as_matrix()
    T[:3, 3] = np.array([tx, ty, tz], dtype=np.float64)
    return T


def scaled_intrinsics(args, width, height):
    sx = width / float(args.original_width)
    sy = height / float(args.original_height)
    K = np.array(
        [
            [args.fx * sx, 0.0, args.cx * sx],
            [0.0, args.fy * sy, args.cy * sy],
            [0.0, 0.0, 1.0],
        ],
        dtype=np.float64,
    )
    return K


def resize_sensor_depth(depth_raw, width, height, depth_factor):
    depth_m = depth_raw.astype(np.float32) / float(depth_factor)
    return cv2.resize(depth_m, (width, height), interpolation=cv2.INTER_NEAREST)


def valid_mask(da3_depth, sensor_depth, min_depth, max_depth):
    valid = np.isfinite(da3_depth) & np.isfinite(sensor_depth)
    valid &= da3_depth > 0
    valid &= sensor_depth >= min_depth
    valid &= sensor_depth <= max_depth
    return valid


def robust_affine_fit(x, y):
    if x.size < 3:
        return 1.0, 0.0

    A = np.stack([x, np.ones_like(x)], axis=1)
    a, b = np.linalg.lstsq(A, y, rcond=None)[0]
    residual = y - (a * x + b)
    keep = np.abs(residual) <= np.percentile(np.abs(residual), 90)
    if np.count_nonzero(keep) >= 3:
        A = np.stack([x[keep], np.ones_like(x[keep])], axis=1)
        a, b = np.linalg.lstsq(A, y[keep], rcond=None)[0]
    return float(a), float(b)


def fit_frame_models(da3_depth, sensor_depth, mask, sample_stride):
    sampled = mask.copy()
    stride_mask = np.zeros_like(mask, dtype=bool)
    stride_mask[::sample_stride, ::sample_stride] = True
    sampled &= stride_mask

    da3 = da3_depth[sampled].astype(np.float64)
    sensor = sensor_depth[sampled].astype(np.float64)
    if da3.size < 3:
        return {
            "scale": 1.0,
            "affine": [1.0, 0.0],
            "inv_affine": [1.0, 0.0],
            "sample_count": int(da3.size),
        }

    ratio = sensor / da3
    ratio = ratio[np.isfinite(ratio) & (ratio > 0)]
    scale = float(np.median(ratio)) if ratio.size else 1.0

    affine_a, affine_b = robust_affine_fit(da3, sensor)

    inv_da3 = 1.0 / da3
    inv_sensor = 1.0 / sensor
    inv_a, inv_b = robust_affine_fit(inv_da3, inv_sensor)

    return {
        "scale": scale,
        "affine": [affine_a, affine_b],
        "inv_affine": [inv_a, inv_b],
        "sample_count": int(da3.size),
    }


def apply_model(da3_depth, model_name, params):
    if model_name == "raw":
        return da3_depth.astype(np.float64)

    if model_name == "scale":
        return params["scale"] * da3_depth

    if model_name == "affine":
        a, b = params["affine"]
        return a * da3_depth + b

    if model_name == "inv_affine":
        a, b = params["inv_affine"]
        inv = a * (1.0 / np.maximum(da3_depth, 1e-6)) + b
        depth = np.full_like(da3_depth, np.nan, dtype=np.float64)
        valid = inv > 1e-6
        depth[valid] = 1.0 / inv[valid]
        return depth

    if model_name == "sensor":
        raise ValueError("Sensor model is handled separately.")

    raise ValueError(f"Unknown model: {model_name}")


def depth_error_stats(depth, sensor_depth, mask):
    valid = mask & np.isfinite(depth) & (depth > 0)
    if not np.any(valid):
        return None
    errors = depth[valid] - sensor_depth[valid]
    abs_errors = np.abs(errors)
    return {
        "count": int(errors.size),
        "rmse": float(np.sqrt(np.mean(errors * errors))),
        "mae": float(np.mean(abs_errors)),
        "median_abs": float(np.median(abs_errors)),
        "p90_abs": float(np.percentile(abs_errors, 90)),
        "bias": float(np.mean(errors)),
    }


def backproject_depth(rgb_bgr, depth, K, T_w_c, stride, min_depth, max_depth):
    h, w = depth.shape
    rgb_bgr = cv2.resize(rgb_bgr, (w, h), interpolation=cv2.INTER_AREA)

    us, vs = np.meshgrid(
        np.arange(0, w, stride),
        np.arange(0, h, stride),
    )
    z = depth[vs, us].astype(np.float64)
    valid = np.isfinite(z) & (z >= min_depth) & (z <= max_depth)

    us = us[valid].astype(np.float64)
    vs = vs[valid].astype(np.float64)
    z = z[valid]

    fx = float(K[0, 0])
    fy = float(K[1, 1])
    cx = float(K[0, 2])
    cy = float(K[1, 2])

    x = (us - cx) * z / fx
    y = (vs - cy) * z / fy
    pts_c = np.stack([x, y, z], axis=1)
    pts_c_h = np.concatenate([pts_c, np.ones((pts_c.shape[0], 1))], axis=1)
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


def aggregate_stats(frame_stats):
    out = {}
    for model_name in ["raw", "scale", "affine", "inv_affine"]:
        vals = [fs["errors"][model_name] for fs in frame_stats if fs["errors"][model_name]]
        if not vals:
            continue
        out[model_name] = {
            key: float(np.mean([v[key] for v in vals]))
            for key in ["rmse", "mae", "median_abs", "p90_abs", "bias"]
        }
        out[model_name]["count"] = int(sum(v["count"] for v in vals))
    return out


def main():
    parser = argparse.ArgumentParser(
        description="Fit DA3 depth to TUM sensor depth and write GT-pose PLY diagnostics."
    )
    parser.add_argument("--da3_npz", required=True)
    parser.add_argument("--sync_csv", required=True)
    parser.add_argument("--rgb_dir", required=True)
    parser.add_argument("--depth_dir", required=True)
    parser.add_argument("--output_dir", required=True)
    parser.add_argument("--start_frame", type=int, required=True)
    parser.add_argument("--end_frame", type=int, required=True)
    parser.add_argument("--da3_start_frame", type=int, default=None)
    parser.add_argument("--stride", type=int, default=6)
    parser.add_argument("--fit_stride", type=int, default=3)
    parser.add_argument("--min_depth", type=float, default=0.2)
    parser.add_argument("--max_depth", type=float, default=5.0)
    parser.add_argument("--depth_factor", type=float, default=5000.0)
    parser.add_argument("--original_width", type=int, default=640)
    parser.add_argument("--original_height", type=int, default=480)
    parser.add_argument("--fx", type=float, default=517.3)
    parser.add_argument("--fy", type=float, default=516.5)
    parser.add_argument("--cx", type=float, default=318.6)
    parser.add_argument("--cy", type=float, default=255.3)
    args = parser.parse_args()

    da3_start = args.da3_start_frame if args.da3_start_frame is not None else args.start_frame
    output_dir = Path(args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)

    da3 = np.load(args.da3_npz)
    depth_all = da3["depth"]
    rows = load_sync_rows(args.sync_csv, args.start_frame, args.end_frame)
    if not rows:
        raise RuntimeError("No sync rows selected.")

    model_points = {name: [] for name in ["raw", "scale", "affine", "inv_affine", "sensor"]}
    model_colors = {name: [] for name in model_points}
    frame_stats = []

    for row in rows:
        frame_id = int(row["frame_id"])
        depth_idx = frame_id - da3_start
        if depth_idx < 0 or depth_idx >= depth_all.shape[0]:
            raise IndexError(
                f"Frame {frame_id} maps to DA3 depth index {depth_idx}; "
                f"DA3 depth shape is {depth_all.shape}."
            )

        da3_depth = depth_all[depth_idx].astype(np.float64)
        h, w = da3_depth.shape
        K = scaled_intrinsics(args, width=w, height=h)

        rgb = cv2.imread(str(Path(args.rgb_dir) / row["filename"]), cv2.IMREAD_COLOR)
        if rgb is None:
            raise FileNotFoundError(Path(args.rgb_dir) / row["filename"])

        depth_raw = cv2.imread(
            str(Path(args.depth_dir) / row["depth_filename"]),
            cv2.IMREAD_UNCHANGED,
        )
        if depth_raw is None:
            raise FileNotFoundError(Path(args.depth_dir) / row["depth_filename"])

        sensor_depth = resize_sensor_depth(depth_raw, w, h, args.depth_factor)
        mask = valid_mask(da3_depth, sensor_depth, args.min_depth, args.max_depth)
        params = fit_frame_models(da3_depth, sensor_depth, mask, args.fit_stride)
        T_w_c = pose_to_matrix(row)

        errors = {}
        for model_name in ["raw", "scale", "affine", "inv_affine"]:
            corrected = apply_model(da3_depth, model_name, params)
            errors[model_name] = depth_error_stats(corrected, sensor_depth, mask)
            pts, cols = backproject_depth(
                rgb,
                corrected,
                K,
                T_w_c,
                args.stride,
                args.min_depth,
                args.max_depth,
            )
            model_points[model_name].append(pts)
            model_colors[model_name].append(cols)

        sensor_errors = depth_error_stats(sensor_depth, sensor_depth, mask)
        errors["sensor"] = sensor_errors
        pts, cols = backproject_depth(
            rgb,
            sensor_depth,
            K,
            T_w_c,
            args.stride,
            args.min_depth,
            args.max_depth,
        )
        model_points["sensor"].append(pts)
        model_colors["sensor"].append(cols)

        frame_stats.append({
            "frame_id": frame_id,
            "depth_idx": depth_idx,
            "fit": params,
            "valid_pixels": int(np.count_nonzero(mask)),
            "errors": errors,
        })
        print(
            f"frame {frame_id}: scale={params['scale']:.6f}, "
            f"affine=({params['affine'][0]:.6f}, {params['affine'][1]:.6f}), "
            f"inv_affine=({params['inv_affine'][0]:.6f}, {params['inv_affine'][1]:.6f})"
        )

    outputs = {}
    for model_name in model_points:
        points = np.concatenate(model_points[model_name], axis=0)
        colors = np.concatenate(model_colors[model_name], axis=0)
        output_path = output_dir / f"pcl_{args.start_frame:03d}_{args.end_frame:03d}_tum_gtpose_da3_{model_name}.ply"
        if model_name == "sensor":
            output_path = output_dir / f"pcl_{args.start_frame:03d}_{args.end_frame:03d}_tum_gtpose_sensor_depth.ply"
        save_ply(output_path, points, colors)
        outputs[model_name] = str(output_path)
        print(f"saved {model_name}: {output_path} ({points.shape[0]} points)")

    report = {
        "da3_npz": args.da3_npz,
        "sync_csv": args.sync_csv,
        "start_frame": args.start_frame,
        "end_frame": args.end_frame,
        "da3_start_frame": da3_start,
        "intrinsics_original": {
            "width": args.original_width,
            "height": args.original_height,
            "fx": args.fx,
            "fy": args.fy,
            "cx": args.cx,
            "cy": args.cy,
        },
        "depth_factor": args.depth_factor,
        "min_depth": args.min_depth,
        "max_depth": args.max_depth,
        "stride": args.stride,
        "fit_stride": args.fit_stride,
        "aggregate_errors": aggregate_stats(frame_stats),
        "frames": frame_stats,
        "outputs": outputs,
    }
    report_path = output_dir / f"pcl_{args.start_frame:03d}_{args.end_frame:03d}_tum_gtpose_da3_depth_anchor_report.json"
    with report_path.open("w") as f:
        json.dump(report, f, indent=2)
        f.write("\n")
    print(f"saved report: {report_path}")


if __name__ == "__main__":
    main()
