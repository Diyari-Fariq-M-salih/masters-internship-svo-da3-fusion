#!/usr/bin/env python3
"""
Refine DA3 chunk-to-chunk overlap alignment with lightweight point ICP.

This is a diagnostic for cases where overlapping DA3-inverted camera centers
align well, but the reconstructed geometry still shows a rotation/scale offset.
It initializes from overlap camera-center Sim(3), builds overlap-only point sets
from the DA3 NPZ files, then refines source -> target with nearest-neighbor
point-to-point Sim(3) iterations.
"""

import argparse
import json
from pathlib import Path

import numpy as np
from scipy.spatial import cKDTree


def da3_extrinsics_to_matrix(E):
    T = np.eye(4, dtype=np.float64)
    T[:3, :4] = E.astype(np.float64)
    return T


def da3_inv_pose(E):
    return np.linalg.inv(da3_extrinsics_to_matrix(E))


def da3_inv_centers(da3_npz, chunk_start, overlap_start, overlap_end):
    da3 = np.load(da3_npz)
    E_all = da3["extrinsics"]

    centers = []
    for frame_id in range(overlap_start, overlap_end + 1):
        depth_idx = frame_id - chunk_start
        if depth_idx < 0 or depth_idx >= E_all.shape[0]:
            raise IndexError(
                f"Frame {frame_id} maps to DA3 index {depth_idx}, "
                f"but {da3_npz} extrinsics have shape {E_all.shape}."
            )
        centers.append(da3_inv_pose(E_all[depth_idx])[:3, 3])

    return np.array(centers, dtype=np.float64)


def estimate_sim3_umeyama(source, target):
    """
    Estimate target ~= scale * rotation @ source + translation.
    """
    if source.shape != target.shape:
        raise ValueError(f"Shape mismatch: source {source.shape}, target {target.shape}")
    if source.ndim != 2 or source.shape[1] != 3:
        raise ValueError(f"Expected Nx3 point arrays, got {source.shape}")
    if source.shape[0] < 3:
        raise ValueError("At least 3 point pairs are required for Sim(3).")

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
        raise ValueError("Source points have zero variance; cannot estimate scale.")

    scale = float(np.trace(np.diag(singular_values) @ sign) / source_variance)
    translation = mu_target - scale * rotation @ mu_source
    return scale, rotation, translation


def compose_sim3(delta_scale, delta_rotation, delta_translation, scale, rotation, translation):
    """
    Compose delta(target <- current_aligned) after current(target <- source).
    """
    composed_scale = delta_scale * scale
    composed_rotation = delta_rotation @ rotation
    composed_translation = delta_scale * delta_rotation @ translation + delta_translation
    return composed_scale, composed_rotation, composed_translation


def transform_points(points, scale, rotation, translation):
    return scale * (rotation @ points.T).T + translation


def backproject_depth(depth, K, T_w_c, stride, max_depth):
    h, w = depth.shape
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
    return (T_w_c @ pts_c_h.T).T[:, :3]


def build_overlap_cloud(da3_npz, chunk_start, overlap_start, overlap_end, stride, max_depth):
    da3 = np.load(da3_npz)
    depth_all = da3["depth"]
    K_all = da3["intrinsics"]
    E_all = da3["extrinsics"]

    all_points = []
    per_frame_counts = {}
    for frame_id in range(overlap_start, overlap_end + 1):
        depth_idx = frame_id - chunk_start
        if depth_idx < 0 or depth_idx >= depth_all.shape[0]:
            raise IndexError(
                f"Frame {frame_id} maps to DA3 index {depth_idx}, "
                f"but {da3_npz} depth has shape {depth_all.shape}."
            )

        T_w_c = da3_inv_pose(E_all[depth_idx])
        pts = backproject_depth(
            depth_all[depth_idx],
            K_all[depth_idx],
            T_w_c,
            stride=stride,
            max_depth=max_depth,
        )
        all_points.append(pts)
        per_frame_counts[str(frame_id)] = int(pts.shape[0])

    return np.concatenate(all_points, axis=0), per_frame_counts


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
                raise ValueError(f"Expected x y z red green blue in {path}, got: {line!r}")
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


def correspondence_stats(distances):
    return {
        "rmse": float(np.sqrt(np.mean(distances * distances))),
        "median": float(np.median(distances)),
        "mean": float(np.mean(distances)),
        "max": float(np.max(distances)),
    }


def sim3_icp(source_points, target_points, init_scale, init_rotation, init_translation, args):
    tree = cKDTree(target_points)
    scale = init_scale
    rotation = init_rotation
    translation = init_translation
    history = []

    for iteration in range(args.icp_iterations):
        aligned = transform_points(source_points, scale, rotation, translation)
        distances, indices = tree.query(aligned, k=1, workers=-1)

        mask = distances <= args.max_corr
        if int(mask.sum()) < args.min_corr:
            cutoff = np.quantile(distances, args.fallback_quantile)
            mask = distances <= cutoff

        if int(mask.sum()) < args.min_corr:
            raise RuntimeError(
                f"ICP iteration {iteration}: only {int(mask.sum())} correspondences."
            )

        delta_scale, delta_rotation, delta_translation = estimate_sim3_umeyama(
            aligned[mask],
            target_points[indices[mask]],
        )

        scale, rotation, translation = compose_sim3(
            delta_scale,
            delta_rotation,
            delta_translation,
            scale,
            rotation,
            translation,
        )

        used_distances = distances[mask]
        history.append(
            {
                "iteration": iteration + 1,
                "correspondences": int(mask.sum()),
                "corr_threshold": float(args.max_corr),
                "delta_scale": float(delta_scale),
                "stats_before_update": correspondence_stats(used_distances),
            }
        )

        if abs(delta_scale - 1.0) < args.scale_tol and np.linalg.norm(delta_translation) < args.translation_tol:
            break

    final_aligned = transform_points(source_points, scale, rotation, translation)
    final_distances, _ = tree.query(final_aligned, k=1, workers=-1)
    final_mask = final_distances <= args.max_corr
    if int(final_mask.sum()) < args.min_corr:
        cutoff = np.quantile(final_distances, args.fallback_quantile)
        final_mask = final_distances <= cutoff

    return scale, rotation, translation, history, final_distances[final_mask], int(final_mask.sum())


def save_report(path, args, init, refined, overlap_counts, history, final_distances, final_corr):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)

    report = {
        "source_ply": args.source_ply,
        "target_ply": args.target_ply,
        "output_ply": args.output_ply,
        "merged_output_ply": args.merged_output_ply,
        "source_da3_npz": args.source_da3_npz,
        "target_da3_npz": args.target_da3_npz,
        "source_start_frame": args.source_start_frame,
        "target_start_frame": args.target_start_frame,
        "overlap_start_frame": args.overlap_start_frame,
        "overlap_end_frame": args.overlap_end_frame,
        "stride": args.stride,
        "max_depth": args.max_depth,
        "max_corr": args.max_corr,
        "init_transform": init,
        "refined_transform": refined,
        "overlap_point_counts": overlap_counts,
        "icp_history": history,
        "final_correspondences": final_corr,
        "final_overlap_nn_error": correspondence_stats(final_distances),
    }

    with path.open("w") as f:
        json.dump(report, f, indent=2)
        f.write("\n")


def main():
    parser = argparse.ArgumentParser(
        description="Refine a DA3 overlap alignment using overlap-only point ICP."
    )
    parser.add_argument("--source_ply", required=True)
    parser.add_argument("--target_ply", required=True)
    parser.add_argument("--output_ply", required=True)
    parser.add_argument("--merged_output_ply", default=None)
    parser.add_argument("--source_da3_npz", required=True)
    parser.add_argument("--target_da3_npz", required=True)
    parser.add_argument("--source_start_frame", type=int, required=True)
    parser.add_argument("--target_start_frame", type=int, required=True)
    parser.add_argument("--overlap_start_frame", type=int, required=True)
    parser.add_argument("--overlap_end_frame", type=int, required=True)
    parser.add_argument("--stride", type=int, default=8)
    parser.add_argument("--max_depth", type=float, default=10.0)
    parser.add_argument("--icp_iterations", type=int, default=20)
    parser.add_argument("--max_corr", type=float, default=0.08)
    parser.add_argument("--min_corr", type=int, default=500)
    parser.add_argument("--fallback_quantile", type=float, default=0.70)
    parser.add_argument("--scale_tol", type=float, default=1e-4)
    parser.add_argument("--translation_tol", type=float, default=1e-4)
    parser.add_argument("--report_json", default=None)
    args = parser.parse_args()

    source_centers = da3_inv_centers(
        args.source_da3_npz,
        args.source_start_frame,
        args.overlap_start_frame,
        args.overlap_end_frame,
    )
    target_centers = da3_inv_centers(
        args.target_da3_npz,
        args.target_start_frame,
        args.overlap_start_frame,
        args.overlap_end_frame,
    )
    init_scale, init_rotation, init_translation = estimate_sim3_umeyama(
        source_centers,
        target_centers,
    )

    source_overlap, source_counts = build_overlap_cloud(
        args.source_da3_npz,
        args.source_start_frame,
        args.overlap_start_frame,
        args.overlap_end_frame,
        stride=args.stride,
        max_depth=args.max_depth,
    )
    target_overlap, target_counts = build_overlap_cloud(
        args.target_da3_npz,
        args.target_start_frame,
        args.overlap_start_frame,
        args.overlap_end_frame,
        stride=args.stride,
        max_depth=args.max_depth,
    )

    refined_scale, refined_rotation, refined_translation, history, final_distances, final_corr = sim3_icp(
        source_overlap,
        target_overlap,
        init_scale,
        init_rotation,
        init_translation,
        args,
    )

    source_points, source_colors = load_ascii_ply(args.source_ply)
    transformed_source = transform_points(
        source_points,
        refined_scale,
        refined_rotation,
        refined_translation,
    )
    save_ascii_ply(args.output_ply, transformed_source, source_colors)

    if args.merged_output_ply is not None:
        target_points, target_colors = load_ascii_ply(args.target_ply)
        merged_points = np.concatenate([target_points, transformed_source], axis=0)
        merged_colors = np.concatenate([target_colors, source_colors], axis=0)
        save_ascii_ply(args.merged_output_ply, merged_points, merged_colors)

    init = {
        "convention": "target = scale * rotation @ source + translation",
        "scale": float(init_scale),
        "rotation": init_rotation.tolist(),
        "translation": init_translation.tolist(),
    }
    refined = {
        "convention": "target = scale * rotation @ source + translation",
        "scale": float(refined_scale),
        "rotation": refined_rotation.tolist(),
        "translation": refined_translation.tolist(),
    }
    overlap_counts = {
        "source": source_counts,
        "target": target_counts,
        "source_total": int(source_overlap.shape[0]),
        "target_total": int(target_overlap.shape[0]),
    }

    if args.report_json is not None:
        save_report(
            args.report_json,
            args,
            init,
            refined,
            overlap_counts,
            history,
            final_distances,
            final_corr,
        )

    print("Refined DA3 overlap Sim(3), source -> target:")
    print(f"  frames: {args.overlap_start_frame}-{args.overlap_end_frame}")
    print(f"  overlap points: source={source_overlap.shape[0]} target={target_overlap.shape[0]}")
    print(f"  init scale: {init_scale:.9f}")
    print(f"  refined scale: {refined_scale:.9f}")
    print(
        "  final overlap NN error: "
        f"rmse={np.sqrt(np.mean(final_distances * final_distances)):.9f}, "
        f"median={np.median(final_distances):.9f}, "
        f"max={np.max(final_distances):.9f}, "
        f"corr={final_corr}"
    )
    print(f"Saved transformed source: {args.output_ply}")
    if args.merged_output_ply is not None:
        print(f"Saved merged diagnostic: {args.merged_output_ply}")
    if args.report_json is not None:
        print(f"Saved report: {args.report_json}")


if __name__ == "__main__":
    main()
