#!/usr/bin/env python3
"""
Align an estimated trajectory to ground truth with Sim(3) and report ATE stats.

Input format is TUM-like:
  timestamp tx ty tz qx qy qz qw
Comment lines beginning with # are ignored.
"""

import argparse
import json
from bisect import bisect_left
from pathlib import Path

import numpy as np


def load_tum(path):
    rows = []
    with Path(path).open("r") as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            vals = line.split()
            if len(vals) < 8:
                continue
            rows.append([float(v) for v in vals[:8]])
    if not rows:
        raise RuntimeError(f"No trajectory rows loaded from {path}")
    return np.array(rows, dtype=np.float64)


def nearest_idx(times, timestamp):
    idx = bisect_left(times, timestamp)
    candidates = []
    if idx < len(times):
        candidates.append(idx)
    if idx > 0:
        candidates.append(idx - 1)
    return min(candidates, key=lambda i: abs(times[i] - timestamp))


def associate(est, gt, max_dt):
    gt_times = gt[:, 0]
    pairs = []
    for est_idx, row in enumerate(est):
        gt_idx = nearest_idx(gt_times, row[0])
        dt = abs(gt_times[gt_idx] - row[0])
        if dt <= max_dt:
            pairs.append((est_idx, gt_idx, dt))
    if not pairs:
        raise RuntimeError("No estimate/GT pairs after timestamp association.")
    return np.array(pairs, dtype=np.float64)


def estimate_sim3_umeyama(source, target):
    """
    Estimate target ~= scale * rotation @ source + translation.
    """
    if source.shape != target.shape:
        raise ValueError(f"Shape mismatch: source {source.shape}, target {target.shape}")
    if source.ndim != 2 or source.shape[1] != 3:
        raise ValueError(f"Expected Nx3 arrays, got {source.shape}")
    if source.shape[0] < 3:
        raise ValueError("At least 3 correspondences are required.")

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
        raise ValueError("Source trajectory has zero variance.")

    scale = float(np.trace(np.diag(singular_values) @ sign) / source_variance)
    translation = mu_target - scale * rotation @ mu_source
    aligned = scale * (rotation @ source.T).T + translation
    errors = np.linalg.norm(aligned - target, axis=1)
    return scale, rotation, translation, aligned, errors


def path_length(points):
    if points.shape[0] < 2:
        return 0.0
    return float(np.linalg.norm(np.diff(points, axis=0), axis=1).sum())


def stats(errors):
    return {
        "rmse": float(np.sqrt(np.mean(errors * errors))),
        "median": float(np.median(errors)),
        "mean": float(np.mean(errors)),
        "p90": float(np.percentile(errors, 90)),
        "p95": float(np.percentile(errors, 95)),
        "max": float(np.max(errors)),
    }


def save_aligned_tum(path, times, aligned_points, quats):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w") as f:
        f.write("# timestamp tx ty tz qx qy qz qw\n")
        for timestamp, point, quat in zip(times, aligned_points, quats):
            f.write(
                f"{timestamp:.9f} "
                f"{point[0]:.9f} {point[1]:.9f} {point[2]:.9f} "
                f"{quat[0]:.9f} {quat[1]:.9f} {quat[2]:.9f} {quat[3]:.9f}\n"
            )


def save_trajectory_plot(path, est_xyz, gt_xyz, aligned_xyz):
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)

    fig, axes = plt.subplots(1, 3, figsize=(15, 4.5), constrained_layout=True)
    views = [
        (0, 1, "x [m]", "y [m]", "XY"),
        (0, 2, "x [m]", "z [m]", "XZ"),
        (1, 2, "y [m]", "z [m]", "YZ"),
    ]
    for ax, (i, j, xlabel, ylabel, title) in zip(axes, views):
        ax.plot(gt_xyz[:, i], gt_xyz[:, j], color="black", linewidth=1.8, label="GT cam0")
        ax.plot(est_xyz[:, i], est_xyz[:, j], color="tab:orange", linewidth=1.0, alpha=0.7, label="SVO raw")
        ax.plot(
            aligned_xyz[:, i],
            aligned_xyz[:, j],
            color="tab:blue",
            linewidth=1.2,
            alpha=0.9,
            label="SVO Sim(3)-aligned",
        )
        ax.set_title(title)
        ax.set_xlabel(xlabel)
        ax.set_ylabel(ylabel)
        ax.axis("equal")
        ax.grid(True, alpha=0.25)
    axes[0].legend(loc="best")
    fig.suptitle("SVO VIO trajectory vs EuRoC cam0 ground truth")
    fig.savefig(path, dpi=180)
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser(
        description="Compare estimated trajectory to GT after Sim(3) alignment."
    )
    parser.add_argument("--estimate", required=True)
    parser.add_argument("--groundtruth", required=True)
    parser.add_argument("--max_dt", type=float, default=0.01)
    parser.add_argument("--report_json", default=None)
    parser.add_argument("--aligned_tum", default=None)
    parser.add_argument("--plot_png", default=None)
    args = parser.parse_args()

    est = load_tum(args.estimate)
    gt = load_tum(args.groundtruth)
    pairs = associate(est, gt, args.max_dt)

    est_indices = pairs[:, 0].astype(np.int64)
    gt_indices = pairs[:, 1].astype(np.int64)
    dts = pairs[:, 2]

    est_xyz = est[est_indices, 1:4]
    gt_xyz = gt[gt_indices, 1:4]

    scale, rotation, translation, aligned, errors = estimate_sim3_umeyama(
        est_xyz,
        gt_xyz,
    )

    est_path = path_length(est_xyz)
    gt_path = path_length(gt_xyz)
    aligned_path = path_length(aligned)
    error_stats = stats(errors)

    report = {
        "estimate": args.estimate,
        "groundtruth": args.groundtruth,
        "max_dt": args.max_dt,
        "count_estimate": int(est.shape[0]),
        "count_groundtruth": int(gt.shape[0]),
        "count_matched": int(len(pairs)),
        "time_range_matched": {
            "start": float(est[est_indices[0], 0]),
            "end": float(est[est_indices[-1], 0]),
            "duration": float(est[est_indices[-1], 0] - est[est_indices[0], 0]),
        },
        "timestamp_dt": {
            "max": float(np.max(dts)),
            "median": float(np.median(dts)),
            "mean": float(np.mean(dts)),
        },
        "path_length": {
            "estimate": est_path,
            "groundtruth": gt_path,
            "aligned_estimate": aligned_path,
            "estimate_over_groundtruth": float(est_path / gt_path) if gt_path > 0 else None,
        },
        "sim3_estimate_to_groundtruth": {
            "scale": scale,
            "rotation": rotation.tolist(),
            "translation": translation.tolist(),
        },
        "ate": error_stats,
        "aligned_tum": args.aligned_tum,
        "plot_png": args.plot_png,
    }

    if args.report_json is not None:
        report_path = Path(args.report_json)
        report_path.parent.mkdir(parents=True, exist_ok=True)
        with report_path.open("w") as f:
            json.dump(report, f, indent=2)
            f.write("\n")

    if args.aligned_tum is not None:
        save_aligned_tum(
            args.aligned_tum,
            est[est_indices, 0],
            aligned,
            est[est_indices, 4:8],
        )
    if args.plot_png is not None:
        save_trajectory_plot(args.plot_png, est_xyz, gt_xyz, aligned)

    print("Trajectory Sim(3) alignment, estimate -> ground truth")
    print(f"  estimate:    {args.estimate}")
    print(f"  groundtruth: {args.groundtruth}")
    print(f"  matched:     {len(pairs)} / {est.shape[0]} estimate poses")
    print(
        "  time range:  "
        f"{report['time_range_matched']['start']:.9f} .. "
        f"{report['time_range_matched']['end']:.9f} "
        f"({report['time_range_matched']['duration']:.3f} s)"
    )
    print(
        "  timestamp dt: "
        f"max={np.max(dts):.9f}, median={np.median(dts):.9f}"
    )
    print("  path length:")
    print(f"    estimate:         {est_path:.9f}")
    print(f"    groundtruth:      {gt_path:.9f}")
    print(f"    aligned estimate: {aligned_path:.9f}")
    if gt_path > 0:
        print(f"    est/gt ratio:     {est_path / gt_path:.9f}")
    print("  Sim(3):")
    print(f"    scale: {scale:.9f}")
    print(
        "    translation: "
        f"{translation[0]: .9f} {translation[1]: .9f} {translation[2]: .9f}"
    )
    print("  ATE:")
    print(
        f"    rmse={error_stats['rmse']:.9f}, "
        f"median={error_stats['median']:.9f}, "
        f"mean={error_stats['mean']:.9f}, "
        f"p95={error_stats['p95']:.9f}, "
        f"max={error_stats['max']:.9f}"
    )
    if args.report_json is not None:
        print(f"  report: {args.report_json}")
    if args.aligned_tum is not None:
        print(f"  aligned_tum: {args.aligned_tum}")
    if args.plot_png is not None:
        print(f"  plot_png: {args.plot_png}")


if __name__ == "__main__":
    main()
