#!/usr/bin/env python3
"""
Align one DA3-inverted chunk to another using shared overlap frames.

Example:
  target chunk: 22-37
  source chunk: 30-45
  overlap:      30-37

The script estimates:
  target_da3_inv_center ~= scale * rotation @ source_da3_inv_center + translation

and applies that Sim(3) to the source chunk PLY.
"""

import argparse
import json
from pathlib import Path

import numpy as np


def da3_extrinsics_to_matrix(E):
    T = np.eye(4, dtype=np.float64)
    T[:3, :4] = E.astype(np.float64)
    return T


def da3_inv_poses(da3_npz, chunk_start, overlap_start, overlap_end):
    da3 = np.load(da3_npz)
    if "extrinsics" not in da3:
        raise KeyError("DA3 npz does not contain 'extrinsics'.")

    E_all = da3["extrinsics"]
    centers = []
    rotations = []
    frame_ids = []

    for frame_id in range(overlap_start, overlap_end + 1):
        depth_idx = frame_id - chunk_start
        if depth_idx < 0 or depth_idx >= E_all.shape[0]:
            raise IndexError(
                f"Frame {frame_id} maps to DA3 index {depth_idx}, "
                f"but {da3_npz} extrinsics have shape {E_all.shape}."
            )

        T_da3 = da3_extrinsics_to_matrix(E_all[depth_idx])
        T_da3_inv = np.linalg.inv(T_da3)
        centers.append(T_da3_inv[:3, 3])
        rotations.append(T_da3_inv[:3, :3])
        frame_ids.append(frame_id)

    return (
        np.array(frame_ids, dtype=np.int64),
        np.array(centers, dtype=np.float64),
        np.array(rotations, dtype=np.float64),
    )


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


def estimate_rotation_from_orientations(source_rotations, target_rotations):
    """
    Estimate target_R ~= rotation @ source_R over matched camera orientations.
    """
    if source_rotations.shape != target_rotations.shape:
        raise ValueError(
            f"Rotation shape mismatch: source {source_rotations.shape}, "
            f"target {target_rotations.shape}"
        )
    if source_rotations.ndim != 3 or source_rotations.shape[1:] != (3, 3):
        raise ValueError(f"Expected Nx3x3 rotations, got {source_rotations.shape}")

    M = np.zeros((3, 3), dtype=np.float64)
    for source_R, target_R in zip(source_rotations, target_rotations):
        M += target_R @ source_R.T

    U, _, Vt = np.linalg.svd(M)
    rotation = U @ Vt
    if np.linalg.det(rotation) < 0:
        U[:, -1] *= -1.0
        rotation = U @ Vt

    return rotation


def estimate_rotation_hybrid(
    source_centers,
    target_centers,
    source_rotations,
    target_rotations,
    orientation_weight,
):
    """
    Estimate rotation from both centered camera positions and orientations.

    Position vectors are normalized by RMS radius so their scale does not swamp
    the unit orientation axes. Orientation axes are weighted by
    orientation_weight.
    """
    if orientation_weight < 0:
        raise ValueError("orientation_weight must be non-negative.")

    source_centered = source_centers - source_centers.mean(axis=0)
    target_centered = target_centers - target_centers.mean(axis=0)

    source_rms = np.sqrt(np.mean(np.sum(source_centered * source_centered, axis=1)))
    target_rms = np.sqrt(np.mean(np.sum(target_centered * target_centered, axis=1)))
    if source_rms <= 0 or target_rms <= 0:
        raise ValueError("Camera centers have zero variance; cannot estimate rotation.")

    source_vectors = [source_centered / source_rms]
    target_vectors = [target_centered / target_rms]

    if orientation_weight > 0:
        for axis_idx in range(3):
            source_vectors.append(orientation_weight * source_rotations[:, :, axis_idx])
            target_vectors.append(orientation_weight * target_rotations[:, :, axis_idx])

    source_stack = np.concatenate(source_vectors, axis=0)
    target_stack = np.concatenate(target_vectors, axis=0)

    M = target_stack.T @ source_stack
    U, _, Vt = np.linalg.svd(M)
    rotation = U @ Vt
    if np.linalg.det(rotation) < 0:
        U[:, -1] *= -1.0
        rotation = U @ Vt

    return rotation


def estimate_scale_translation_with_fixed_rotation(source, target, rotation):
    """
    Estimate target ~= scale * rotation @ source + translation with fixed rotation.
    """
    if source.shape != target.shape:
        raise ValueError(f"Shape mismatch: source {source.shape}, target {target.shape}")

    mu_source = source.mean(axis=0)
    mu_target = target.mean(axis=0)
    X = source - mu_source
    Y = target - mu_target
    RX = (rotation @ X.T).T

    denom = float(np.sum(RX * RX))
    if denom <= 0:
        raise ValueError("Source centers have zero variance; cannot estimate scale.")

    scale = float(np.sum(Y * RX) / denom)
    translation = mu_target - scale * rotation @ mu_source
    aligned = scale * (rotation @ source.T).T + translation
    errors = np.linalg.norm(aligned - target, axis=1)

    return scale, translation, errors


def estimate_translation_with_fixed_rotation_scale(source, target, rotation, scale):
    """
    Estimate target ~= scale * rotation @ source + translation with fixed
    rotation and fixed scale.
    """
    if source.shape != target.shape:
        raise ValueError(f"Shape mismatch: source {source.shape}, target {target.shape}")

    mu_source = source.mean(axis=0)
    mu_target = target.mean(axis=0)
    translation = mu_target - scale * rotation @ mu_source
    aligned = scale * (rotation @ source.T).T + translation
    errors = np.linalg.norm(aligned - target, axis=1)

    return translation, errors


def rotation_errors_degrees(rotation, source_rotations, target_rotations):
    errors = []
    for source_R, target_R in zip(source_rotations, target_rotations):
        residual = target_R @ (rotation @ source_R).T
        cos_angle = (np.trace(residual) - 1.0) / 2.0
        cos_angle = np.clip(cos_angle, -1.0, 1.0)
        errors.append(np.degrees(np.arccos(cos_angle)))
    return np.array(errors, dtype=np.float64)


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


def save_report(
    path,
    args,
    scale,
    rotation,
    translation,
    errors,
    rotation_errors,
    frame_ids,
    point_count,
):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)

    report = {
        "source_ply": args.source_ply,
        "output_ply": args.output_ply,
        "source_da3_npz": args.source_da3_npz,
        "target_da3_npz": args.target_da3_npz,
        "source_start_frame": args.source_start_frame,
        "target_start_frame": args.target_start_frame,
        "overlap_start_frame": args.overlap_start_frame,
        "overlap_end_frame": args.overlap_end_frame,
        "overlap_frame_ids": frame_ids.tolist(),
        "rotation_mode": args.rotation_mode,
        "scale_override": args.scale_override,
        "transform": {
            "convention": "target_center = scale * rotation @ source_center + translation",
            "scale": scale,
            "rotation": rotation.tolist(),
            "translation": translation.tolist(),
        },
        "alignment_error": {
            "rmse": float(np.sqrt(np.mean(errors * errors))),
            "median": float(np.median(errors)),
            "mean": float(np.mean(errors)),
            "max": float(np.max(errors)),
        },
        "rotation_error_degrees": {
            "rmse": float(np.sqrt(np.mean(rotation_errors * rotation_errors))),
            "median": float(np.median(rotation_errors)),
            "mean": float(np.mean(rotation_errors)),
            "max": float(np.max(rotation_errors)),
        },
        "point_count": int(point_count),
    }

    with path.open("w") as f:
        json.dump(report, f, indent=2)
        f.write("\n")


def main():
    parser = argparse.ArgumentParser(
        description="Align a DA3-inv source chunk PLY to a DA3-inv target chunk using overlap frames."
    )
    parser.add_argument("--source_ply", required=True)
    parser.add_argument("--output_ply", required=True)
    parser.add_argument("--source_da3_npz", required=True)
    parser.add_argument("--target_da3_npz", required=True)
    parser.add_argument("--source_start_frame", type=int, required=True)
    parser.add_argument("--target_start_frame", type=int, required=True)
    parser.add_argument("--overlap_start_frame", type=int, required=True)
    parser.add_argument("--overlap_end_frame", type=int, required=True)
    parser.add_argument(
        "--rotation_mode",
        choices=["centers", "orientations", "hybrid"],
        default="centers",
        help=(
            "centers: estimate full Sim(3) from camera centers. "
            "orientations: estimate rotation from DA3-inv camera orientations, "
            "then estimate scale/translation from centers. "
            "hybrid: estimate rotation from both normalized center motion and "
            "camera orientations, then estimate scale/translation from centers."
        ),
    )
    parser.add_argument(
        "--orientation_weight",
        type=float,
        default=1.0,
        help="Orientation-axis weight used by --rotation_mode hybrid.",
    )
    parser.add_argument(
        "--scale_override",
        type=float,
        default=None,
        help=(
            "If set, replace the estimated Sim(3) scale with this value and "
            "recompute translation from overlap camera centers. Use 1.0 to "
            "test a scale-preserving merge."
        ),
    )
    parser.add_argument("--report_json", default=None)
    args = parser.parse_args()

    source_frame_ids, source_centers, source_rotations = da3_inv_poses(
        args.source_da3_npz,
        args.source_start_frame,
        args.overlap_start_frame,
        args.overlap_end_frame,
    )
    target_frame_ids, target_centers, target_rotations = da3_inv_poses(
        args.target_da3_npz,
        args.target_start_frame,
        args.overlap_start_frame,
        args.overlap_end_frame,
    )
    if not np.array_equal(source_frame_ids, target_frame_ids):
        raise RuntimeError("Source and target overlap frame IDs do not match.")

    if args.rotation_mode == "centers":
        scale, rotation, translation, errors = estimate_sim3_umeyama(
            source=source_centers,
            target=target_centers,
        )
    elif args.rotation_mode == "orientations":
        rotation = estimate_rotation_from_orientations(
            source_rotations,
            target_rotations,
        )
        scale, translation, errors = estimate_scale_translation_with_fixed_rotation(
            source_centers,
            target_centers,
            rotation,
        )
    else:
        rotation = estimate_rotation_hybrid(
            source_centers,
            target_centers,
            source_rotations,
            target_rotations,
            args.orientation_weight,
        )
        scale, translation, errors = estimate_scale_translation_with_fixed_rotation(
            source_centers,
            target_centers,
            rotation,
        )

    if args.scale_override is not None:
        scale = float(args.scale_override)
        translation, errors = estimate_translation_with_fixed_rotation_scale(
            source_centers,
            target_centers,
            rotation,
            scale,
        )

    rotation_errors = rotation_errors_degrees(
        rotation,
        source_rotations,
        target_rotations,
    )

    points, colors = load_ascii_ply(args.source_ply)
    aligned_points = scale * (rotation @ points.T).T + translation
    save_ascii_ply(args.output_ply, aligned_points, colors)

    if args.report_json is not None:
        save_report(
            args.report_json,
            args,
            scale,
            rotation,
            translation,
            errors,
            rotation_errors,
            source_frame_ids,
            points.shape[0],
        )

    print("Estimated overlap Sim(3), source -> target:")
    print(f"  frames: {args.overlap_start_frame}-{args.overlap_end_frame}")
    print(f"  rotation_mode: {args.rotation_mode}")
    if args.scale_override is not None:
        print(f"  scale_override: {args.scale_override:.9f}")
    print(f"  scale: {scale:.9f}")
    print(
        "  center alignment error: "
        f"rmse={np.sqrt(np.mean(errors * errors)):.9f}, "
        f"median={np.median(errors):.9f}, "
        f"max={np.max(errors):.9f}"
    )
    print(
        "  rotation alignment error (deg): "
        f"rmse={np.sqrt(np.mean(rotation_errors * rotation_errors)):.9f}, "
        f"median={np.median(rotation_errors):.9f}, "
        f"max={np.max(rotation_errors):.9f}"
    )
    print(f"Transformed points: {points.shape[0]}")
    print(f"Saved: {args.output_ply}")
    if args.report_json is not None:
        print(f"Saved report: {args.report_json}")


if __name__ == "__main__":
    main()
