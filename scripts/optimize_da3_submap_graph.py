#!/usr/bin/env python3
"""
Optimize DA3-inverted chunks as a global Sim(3) submap graph.

This is a small "GCS-lite" experiment inspired by submap graph fusion:
- each DA3 chunk is a graph node
- every sufficiently overlapping chunk pair becomes an edge
- global Sim(3) transforms are optimized so shared DA3-inverted camera centers
  agree in a common root frame

The optimizer does not implement full GCS-SLAM. It is a lightweight diagnostic
that tests whether global submap optimization improves over greedy pairwise
chain composition.
"""

import argparse
import csv
import json
from dataclasses import dataclass
from pathlib import Path

import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation as Rotation

from align_da3_overlap_ply import (
    da3_inv_poses,
    estimate_sim3_umeyama,
    load_ascii_ply,
    save_ascii_ply,
)


@dataclass
class Chunk:
    start: int
    end: int
    da3_npz: Path
    ply: Path

    @property
    def label(self):
        return f"{self.start:03d}_{self.end:03d}"


@dataclass
class Edge:
    target_idx: int
    source_idx: int
    overlap_start: int
    overlap_end: int
    frame_ids: np.ndarray
    target_centers: np.ndarray
    source_centers: np.ndarray
    target_rotations: np.ndarray
    source_rotations: np.ndarray
    relative_scale: float
    relative_rotation: np.ndarray
    relative_translation: np.ndarray
    initial_errors: np.ndarray
    relative_source: str
    correspondence_count: int


def parse_chunk(spec):
    parts = spec.split(":", 3)
    if len(parts) != 4:
        raise argparse.ArgumentTypeError(
            "Chunk must be START:END:DA3_NPZ:PLY, "
            f"got {spec!r}"
        )
    start_s, end_s, da3_npz_s, ply_s = parts
    return Chunk(
        start=int(start_s),
        end=int(end_s),
        da3_npz=Path(da3_npz_s),
        ply=Path(ply_s),
    )


def load_svo_centers(sync_csv, max_dt):
    centers = {}
    with Path(sync_csv).open("r") as f:
        reader = csv.DictReader(f)
        for row in reader:
            frame_id = int(row["frame_id"])
            dt = float(row["dt"])
            if dt > max_dt:
                continue
            centers[frame_id] = np.array(
                [float(row["tx"]), float(row["ty"]), float(row["tz"])],
                dtype=np.float64,
            )
    return centers


def da3_extrinsics_to_matrix(E):
    T = np.eye(4, dtype=np.float64)
    T[:3, :4] = E.astype(np.float64)
    return T


def backproject_depth_points(depth, K, E, stride, max_depth, conf=None, min_conf=None):
    h, w = depth.shape
    us, vs = np.meshgrid(
        np.arange(0, w, stride),
        np.arange(0, h, stride),
    )

    z = depth[vs, us].astype(np.float64)
    valid = np.isfinite(z) & (z > 0)
    if max_depth is not None:
        valid &= z < max_depth
    if conf is not None and min_conf is not None:
        valid &= conf[vs, us] >= min_conf

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

    T_inv = np.linalg.inv(da3_extrinsics_to_matrix(E))
    return (T_inv @ pts_c_h.T).T[:, :3], valid


def dense_same_pixel_correspondences(
    source,
    target,
    overlap_start,
    overlap_end,
    dense_stride,
    max_depth,
    min_conf,
):
    source_da3 = np.load(source.da3_npz)
    target_da3 = np.load(target.da3_npz)

    source_depth = source_da3["depth"]
    target_depth = target_da3["depth"]
    source_K = source_da3["intrinsics"]
    target_K = target_da3["intrinsics"]
    source_E = source_da3["extrinsics"]
    target_E = target_da3["extrinsics"]
    source_conf = source_da3["conf"] if "conf" in source_da3 else None
    target_conf = target_da3["conf"] if "conf" in target_da3 else None

    source_points = []
    target_points = []

    for frame_id in range(overlap_start, overlap_end + 1):
        source_idx = frame_id - source.start
        target_idx = frame_id - target.start

        if source_depth[source_idx].shape != target_depth[target_idx].shape:
            raise ValueError(
                "Dense same-pixel edge mode requires matching DA3 depth shapes; "
                f"{source.label} frame {frame_id} has {source_depth[source_idx].shape}, "
                f"{target.label} has {target_depth[target_idx].shape}."
            )

        h, w = source_depth[source_idx].shape
        us, vs = np.meshgrid(
            np.arange(0, w, dense_stride),
            np.arange(0, h, dense_stride),
        )

        source_z = source_depth[source_idx][vs, us].astype(np.float64)
        target_z = target_depth[target_idx][vs, us].astype(np.float64)
        valid = (
            np.isfinite(source_z)
            & np.isfinite(target_z)
            & (source_z > 0)
            & (target_z > 0)
        )
        if max_depth is not None:
            valid &= (source_z < max_depth) & (target_z < max_depth)
        if min_conf is not None and source_conf is not None and target_conf is not None:
            valid &= (
                source_conf[source_idx][vs, us] >= min_conf
            ) & (
                target_conf[target_idx][vs, us] >= min_conf
            )

        us_valid = us[valid].astype(np.float64)
        vs_valid = vs[valid].astype(np.float64)
        if us_valid.size == 0:
            continue

        frame_source_points = pixels_to_local_points(
            us_valid,
            vs_valid,
            source_z[valid],
            source_K[source_idx],
            source_E[source_idx],
        )
        frame_target_points = pixels_to_local_points(
            us_valid,
            vs_valid,
            target_z[valid],
            target_K[target_idx],
            target_E[target_idx],
        )
        source_points.append(frame_source_points)
        target_points.append(frame_target_points)

    if not source_points:
        return np.empty((0, 3)), np.empty((0, 3))

    return np.concatenate(source_points, axis=0), np.concatenate(target_points, axis=0)


def pixels_to_local_points(us, vs, z, K, E):
    fx = float(K[0, 0])
    fy = float(K[1, 1])
    cx = float(K[0, 2])
    cy = float(K[1, 2])

    x = (us - cx) * z / fx
    y = (vs - cy) * z / fy
    pts_c = np.stack([x, y, z], axis=1)
    pts_c_h = np.concatenate([pts_c, np.ones((pts_c.shape[0], 1))], axis=1)
    T_inv = np.linalg.inv(da3_extrinsics_to_matrix(E))
    return (T_inv @ pts_c_h.T).T[:, :3]


def compose(parent, child):
    """Compose two Sim(3)s: parent(child(x))."""
    parent_scale, parent_rotation, parent_translation = parent
    child_scale, child_rotation, child_translation = child
    return (
        parent_scale * child_scale,
        parent_rotation @ child_rotation,
        parent_scale * (parent_rotation @ child_translation) + parent_translation,
    )


def apply_transform(points, transform):
    scale, rotation, translation = transform
    return scale * (rotation @ points.T).T + translation


def make_svo_anchors(chunks, sync_csv, max_dt):
    if sync_csv is None:
        return [], None

    svo_centers_by_frame = load_svo_centers(sync_csv, max_dt)
    if not svo_centers_by_frame:
        raise ValueError(f"No SVO anchor centers found in {sync_csv}.")

    root = chunks[0]
    root_frame_ids, root_da3_centers, _ = da3_inv_poses(
        root.da3_npz,
        root.start,
        root.start,
        root.end,
    )

    root_svo_centers = []
    root_da3_matched = []
    for frame_id, da3_center in zip(root_frame_ids, root_da3_centers):
        if int(frame_id) not in svo_centers_by_frame:
            continue
        root_svo_centers.append(svo_centers_by_frame[int(frame_id)])
        root_da3_matched.append(da3_center)

    if len(root_svo_centers) < 3:
        raise ValueError("Need at least 3 root chunk SVO anchors to define root frame.")

    root_scale, root_rotation, root_translation, root_errors = estimate_sim3_umeyama(
        source=np.array(root_svo_centers, dtype=np.float64),
        target=np.array(root_da3_matched, dtype=np.float64),
    )
    svo_to_root = (root_scale, root_rotation, root_translation)

    anchors = []
    for chunk_idx, chunk in enumerate(chunks):
        frame_ids, da3_centers, _ = da3_inv_poses(
            chunk.da3_npz,
            chunk.start,
            chunk.start,
            chunk.end,
        )
        local_centers = []
        target_centers = []
        kept_frame_ids = []
        for frame_id, da3_center in zip(frame_ids, da3_centers):
            frame_id = int(frame_id)
            if frame_id not in svo_centers_by_frame:
                continue
            local_centers.append(da3_center)
            target_centers.append(
                apply_transform(
                    svo_centers_by_frame[frame_id][None, :],
                    svo_to_root,
                )[0]
            )
            kept_frame_ids.append(frame_id)

        if len(local_centers) < 3:
            continue

        anchors.append({
            "chunk_idx": chunk_idx,
            "frame_ids": np.array(kept_frame_ids, dtype=np.int64),
            "local_centers": np.array(local_centers, dtype=np.float64),
            "target_centers": np.array(target_centers, dtype=np.float64),
        })

    return anchors, {
        "svo_to_root": svo_to_root,
        "root_anchor_rmse": float(np.sqrt(np.mean(root_errors * root_errors))),
        "root_anchor_median": float(np.median(root_errors)),
        "root_anchor_count": len(root_svo_centers),
    }


def make_edges(chunks, args):
    edges = []
    for target_idx in range(len(chunks)):
        target = chunks[target_idx]
        for source_idx in range(target_idx + 1, len(chunks)):
            source = chunks[source_idx]
            overlap_start = max(target.start, source.start)
            overlap_end = min(target.end, source.end)
            if overlap_end - overlap_start + 1 < args.min_overlap:
                continue

            source_frame_ids, source_centers, source_rotations = da3_inv_poses(
                source.da3_npz,
                source.start,
                overlap_start,
                overlap_end,
            )
            target_frame_ids, target_centers, target_rotations = da3_inv_poses(
                target.da3_npz,
                target.start,
                overlap_start,
                overlap_end,
            )
            if not np.array_equal(source_frame_ids, target_frame_ids):
                raise RuntimeError("Source and target overlap frame IDs do not match.")

            relative_source = "camera_centers"
            correspondence_count = source_centers.shape[0]

            if args.edge_mode == "dense_same_pixel":
                dense_source, dense_target = dense_same_pixel_correspondences(
                    source,
                    target,
                    overlap_start,
                    overlap_end,
                    args.dense_stride,
                    args.max_depth,
                    args.min_conf,
                )
                if dense_source.shape[0] >= 3:
                    scale, rotation, translation, errors = estimate_sim3_umeyama(
                        source=dense_source,
                        target=dense_target,
                    )
                    relative_source = "dense_same_pixel"
                    correspondence_count = dense_source.shape[0]
                else:
                    scale, rotation, translation, errors = estimate_sim3_umeyama(
                        source=source_centers,
                        target=target_centers,
                    )
            else:
                scale, rotation, translation, errors = estimate_sim3_umeyama(
                    source=source_centers,
                    target=target_centers,
                )

            edges.append(
                Edge(
                    target_idx=target_idx,
                    source_idx=source_idx,
                    overlap_start=overlap_start,
                    overlap_end=overlap_end,
                    frame_ids=source_frame_ids,
                    target_centers=target_centers,
                    source_centers=source_centers,
                    target_rotations=target_rotations,
                    source_rotations=source_rotations,
                    relative_scale=scale,
                    relative_rotation=rotation,
                    relative_translation=translation,
                    initial_errors=errors,
                    relative_source=relative_source,
                    correspondence_count=correspondence_count,
                )
            )

    return edges


def initial_global_transforms(chunks, edges):
    transforms = [(1.0, np.eye(3), np.zeros(3))]
    edge_lookup = {
        (edge.target_idx, edge.source_idx): edge
        for edge in edges
    }

    for idx in range(1, len(chunks)):
        previous_idx = idx - 1
        edge = edge_lookup.get((previous_idx, idx))
        if edge is None:
            raise ValueError(
                f"No adjacent edge found for {chunks[idx].label} -> "
                f"{chunks[previous_idx].label}. Check chunk order/overlap."
            )
        relative = (
            edge.relative_scale,
            edge.relative_rotation,
            edge.relative_translation,
        )
        transforms.append(compose(transforms[previous_idx], relative))

    return transforms


def pack_transforms(transforms):
    params = []
    for scale, rotation, translation in transforms[1:]:
        params.append(np.log(scale))
        params.extend(Rotation.from_matrix(rotation).as_rotvec())
        params.extend(translation)
    return np.array(params, dtype=np.float64)


def unpack_params(params, node_count):
    transforms = [(1.0, np.eye(3), np.zeros(3))]
    cursor = 0
    for _ in range(1, node_count):
        log_scale = params[cursor]
        rotvec = params[cursor + 1 : cursor + 4]
        translation = params[cursor + 4 : cursor + 7]
        cursor += 7
        transforms.append((
            float(np.exp(log_scale)),
            Rotation.from_rotvec(rotvec).as_matrix(),
            translation.copy(),
        ))
    return transforms


def sim3_pose_edge_residual(edge, transforms):
    """
    Approximate Sim(3) pose-graph residual.

    Edge convention:
      target_local ~= T_rel(source_local)

    Global transforms map local coordinates into the root frame. Therefore:
      G_source ~= G_target o T_rel
    """
    target_scale, target_rotation, target_translation = transforms[edge.target_idx]
    source_scale, source_rotation, source_translation = transforms[edge.source_idx]

    expected_scale = target_scale * edge.relative_scale
    expected_rotation = target_rotation @ edge.relative_rotation
    expected_translation = (
        target_scale * (target_rotation @ edge.relative_translation)
        + target_translation
    )

    scale_residual = np.array([np.log(source_scale) - np.log(expected_scale)])
    rotation_residual = Rotation.from_matrix(
        source_rotation @ expected_rotation.T
    ).as_rotvec()
    translation_residual = source_translation - expected_translation

    return np.concatenate([scale_residual, rotation_residual, translation_residual])


def residuals(
    params,
    node_count,
    edges,
    anchors,
    center_weight,
    pose_edge_weight,
    svo_anchor_weight,
    orientation_weight,
    scale_prior_weight,
    initial_log_scales,
):
    transforms = unpack_params(params, node_count)
    residual_chunks = []

    for edge in edges:
        target_transform = transforms[edge.target_idx]
        source_transform = transforms[edge.source_idx]

        target_global = apply_transform(edge.target_centers, target_transform)
        source_global = apply_transform(edge.source_centers, source_transform)

        # Normalize by sqrt(count) so long overlaps do not dominate only because
        # they have more frames.
        center_scale = np.sqrt(max(1, edge.frame_ids.shape[0]))
        if center_weight > 0:
            residual_chunks.append(
                center_weight * ((source_global - target_global) / center_scale).ravel()
            )

        if pose_edge_weight > 0:
            residual_chunks.append(
                pose_edge_weight * sim3_pose_edge_residual(edge, transforms)
            )

        if orientation_weight > 0:
            target_R_global = target_transform[1]
            source_R_global = source_transform[1]
            rot_res = []
            for target_R, source_R in zip(edge.target_rotations, edge.source_rotations):
                residual_R = source_R_global @ source_R @ (target_R_global @ target_R).T
                rot_res.append(Rotation.from_matrix(residual_R).as_rotvec())
            residual_chunks.append(
                orientation_weight * np.array(rot_res).ravel() / center_scale
            )

    if svo_anchor_weight > 0:
        for anchor in anchors:
            transform = transforms[anchor["chunk_idx"]]
            transformed_centers = apply_transform(anchor["local_centers"], transform)
            anchor_scale = np.sqrt(max(1, anchor["local_centers"].shape[0]))
            residual_chunks.append(
                svo_anchor_weight
                * ((transformed_centers - anchor["target_centers"]) / anchor_scale).ravel()
            )

    if scale_prior_weight > 0:
        log_scales = params[0::7]
        residual_chunks.append(scale_prior_weight * (log_scales - initial_log_scales))

    return np.concatenate(residual_chunks)


def anchor_stats(anchors, transforms):
    rows = []
    all_errors = []
    for anchor in anchors:
        transformed_centers = apply_transform(
            anchor["local_centers"],
            transforms[anchor["chunk_idx"]],
        )
        errors = np.linalg.norm(transformed_centers - anchor["target_centers"], axis=1)
        all_errors.append(errors)
        rows.append({
            "chunk": int(anchor["chunk_idx"]),
            "frame_ids": anchor["frame_ids"].tolist(),
            "rmse": float(np.sqrt(np.mean(errors * errors))),
            "median": float(np.median(errors)),
            "max": float(np.max(errors)),
        })

    if not all_errors:
        return rows, np.array([], dtype=np.float64)
    return rows, np.concatenate(all_errors)


def edge_stats(edges, transforms):
    rows = []
    all_errors = []
    for edge in edges:
        target_global = apply_transform(edge.target_centers, transforms[edge.target_idx])
        source_global = apply_transform(edge.source_centers, transforms[edge.source_idx])
        errors = np.linalg.norm(source_global - target_global, axis=1)
        all_errors.append(errors)
        rows.append({
            "target": edge.target_idx,
            "source": edge.source_idx,
            "overlap_start_frame": edge.overlap_start,
            "overlap_end_frame": edge.overlap_end,
            "overlap_frame_ids": edge.frame_ids.tolist(),
            "relative_source": edge.relative_source,
            "correspondence_count": int(edge.correspondence_count),
            "relative_scale_source_to_target": float(edge.relative_scale),
            "pairwise_center_rmse": float(np.sqrt(np.mean(edge.initial_errors * edge.initial_errors))),
            "optimized_center_rmse": float(np.sqrt(np.mean(errors * errors))),
            "optimized_center_median": float(np.median(errors)),
            "optimized_center_max": float(np.max(errors)),
        })
    all_errors = np.concatenate(all_errors) if all_errors else np.array([], dtype=np.float64)
    return rows, all_errors


def transform_to_json(transform):
    scale, rotation, translation = transform
    return {
        "scale": float(scale),
        "rotation": rotation.tolist(),
        "translation": translation.tolist(),
    }


def main():
    parser = argparse.ArgumentParser(
        description="Optimize DA3-inverted chunks with a global Sim(3) submap graph."
    )
    parser.add_argument(
        "--chunk",
        action="append",
        type=parse_chunk,
        required=True,
        help="Chunk as START:END:DA3_NPZ:PLY. Repeat in graph order.",
    )
    parser.add_argument("--output", required=True)
    parser.add_argument("--report_json", default=None)
    parser.add_argument("--save_transformed_dir", default=None)
    parser.add_argument("--min_overlap", type=int, default=3)
    parser.add_argument(
        "--edge_mode",
        choices=["camera_centers", "dense_same_pixel"],
        default="camera_centers",
        help=(
            "camera_centers: estimate relative Sim(3) edges from DA3 camera centers. "
            "dense_same_pixel: estimate edges from sampled same-pixel 3D pairs in "
            "overlap frames, falling back to camera centers if too few pairs exist."
        ),
    )
    parser.add_argument("--dense_stride", type=int, default=24)
    parser.add_argument("--max_depth", type=float, default=10.0)
    parser.add_argument("--min_conf", type=float, default=None)
    parser.add_argument(
        "--svo_anchor_sync_csv",
        default=None,
        help=(
            "Optional sync CSV with SVO poses. If set, DA3-inverted camera "
            "centers are anchored to SVO centers transformed into the root "
            "chunk frame."
        ),
    )
    parser.add_argument("--svo_anchor_weight", type=float, default=0.0)
    parser.add_argument("--svo_anchor_max_dt", type=float, default=0.01)
    parser.add_argument(
        "--center_weight",
        type=float,
        default=1.0,
        help="Weight for shared camera-center agreement residuals.",
    )
    parser.add_argument(
        "--pose_edge_weight",
        type=float,
        default=1.0,
        help=(
            "Weight for relative Sim(3) pose-graph residuals. This prevents "
            "the center-only graph from collapsing submap scales."
        ),
    )
    parser.add_argument(
        "--orientation_weight",
        type=float,
        default=0.0,
        help="Optional rotation residual weight. Start with 0.0 for center-only graph.",
    )
    parser.add_argument(
        "--scale_prior_weight",
        type=float,
        default=0.0,
        help="Optional soft prior preserving greedy-chain log scales.",
    )
    args = parser.parse_args()

    chunks = args.chunk
    if len(chunks) < 2:
        raise ValueError("At least two chunks are required.")

    edges = make_edges(chunks, args)
    if not edges:
        raise ValueError("No overlap edges found.")
    anchors, anchor_root_info = make_svo_anchors(
        chunks,
        args.svo_anchor_sync_csv,
        args.svo_anchor_max_dt,
    )

    initial_transforms = initial_global_transforms(chunks, edges)
    x0 = pack_transforms(initial_transforms)
    initial_log_scales = x0[0::7].copy()

    initial_residual = residuals(
        x0,
        len(chunks),
        edges,
        anchors,
        args.center_weight,
        args.pose_edge_weight,
        args.svo_anchor_weight,
        args.orientation_weight,
        args.scale_prior_weight,
        initial_log_scales,
    )
    print(f"Chunks: {len(chunks)}")
    print(f"Edges: {len(edges)}")
    print(f"Initial residual RMSE: {np.sqrt(np.mean(initial_residual * initial_residual)):.9f}")

    result = least_squares(
        residuals,
        x0,
        args=(
            len(chunks),
            edges,
            anchors,
            args.center_weight,
            args.pose_edge_weight,
            args.svo_anchor_weight,
            args.orientation_weight,
            args.scale_prior_weight,
            initial_log_scales,
        ),
        loss="soft_l1",
        f_scale=0.05,
        max_nfev=1000,
        verbose=1,
    )

    optimized_transforms = unpack_params(result.x, len(chunks))
    initial_edge_rows, initial_edge_errors = edge_stats(edges, initial_transforms)
    optimized_edge_rows, optimized_edge_errors = edge_stats(edges, optimized_transforms)
    initial_anchor_rows, initial_anchor_errors = anchor_stats(anchors, initial_transforms)
    optimized_anchor_rows, optimized_anchor_errors = anchor_stats(anchors, optimized_transforms)

    print(
        "Initial edge center RMSE: "
        f"{np.sqrt(np.mean(initial_edge_errors * initial_edge_errors)):.9f}"
    )
    print(
        "Optimized edge center RMSE: "
        f"{np.sqrt(np.mean(optimized_edge_errors * optimized_edge_errors)):.9f}"
    )
    print(
        "Global scale range: "
        f"{min(t[0] for t in optimized_transforms):.9f} - "
        f"{max(t[0] for t in optimized_transforms):.9f}; "
        f"final={optimized_transforms[-1][0]:.9f}"
    )
    if anchors:
        print(
            "SVO anchor RMSE: "
            f"{np.sqrt(np.mean(initial_anchor_errors * initial_anchor_errors)):.9f} -> "
            f"{np.sqrt(np.mean(optimized_anchor_errors * optimized_anchor_errors)):.9f}"
        )

    transformed_dir = Path(args.save_transformed_dir) if args.save_transformed_dir else None
    if transformed_dir is not None:
        transformed_dir.mkdir(parents=True, exist_ok=True)

    all_points = []
    all_colors = []
    for idx, chunk in enumerate(chunks):
        points, colors = load_ascii_ply(chunk.ply)
        transformed_points = apply_transform(points, optimized_transforms[idx])
        all_points.append(transformed_points)
        all_colors.append(colors)

        if transformed_dir is not None:
            transformed_path = transformed_dir / f"pcl_{chunk.label}_graph_global.ply"
            save_ascii_ply(transformed_path, transformed_points, colors)

    merged_points = np.concatenate(all_points, axis=0)
    merged_colors = np.concatenate(all_colors, axis=0)
    save_ascii_ply(args.output, merged_points, merged_colors)
    print(f"Saved merged cloud: {args.output}")
    print(f"Total points: {merged_points.shape[0]}")

    if args.report_json is not None:
        report = {
            "output": args.output,
            "min_overlap": args.min_overlap,
            "edge_mode": args.edge_mode,
            "dense_stride": args.dense_stride,
            "max_depth": args.max_depth,
            "min_conf": args.min_conf,
            "svo_anchor_sync_csv": args.svo_anchor_sync_csv,
            "svo_anchor_weight": args.svo_anchor_weight,
            "svo_anchor_max_dt": args.svo_anchor_max_dt,
            "anchor_root_info": None if anchor_root_info is None else {
                "svo_to_root": transform_to_json(anchor_root_info["svo_to_root"]),
                "root_anchor_rmse": anchor_root_info["root_anchor_rmse"],
                "root_anchor_median": anchor_root_info["root_anchor_median"],
                "root_anchor_count": anchor_root_info["root_anchor_count"],
            },
            "center_weight": args.center_weight,
            "pose_edge_weight": args.pose_edge_weight,
            "orientation_weight": args.orientation_weight,
            "scale_prior_weight": args.scale_prior_weight,
            "optimizer": {
                "success": bool(result.success),
                "message": result.message,
                "cost": float(result.cost),
                "nfev": int(result.nfev),
                "optimality": float(result.optimality),
            },
            "edge_count": len(edges),
            "initial_edge_center_rmse": float(np.sqrt(np.mean(initial_edge_errors * initial_edge_errors))),
            "optimized_edge_center_rmse": float(np.sqrt(np.mean(optimized_edge_errors * optimized_edge_errors))),
            "optimized_edge_center_median": float(np.median(optimized_edge_errors)),
            "optimized_edge_center_max": float(np.max(optimized_edge_errors)),
            "initial_svo_anchor_rmse": (
                None
                if initial_anchor_errors.size == 0
                else float(np.sqrt(np.mean(initial_anchor_errors * initial_anchor_errors)))
            ),
            "optimized_svo_anchor_rmse": (
                None
                if optimized_anchor_errors.size == 0
                else float(np.sqrt(np.mean(optimized_anchor_errors * optimized_anchor_errors)))
            ),
            "optimized_svo_anchor_median": (
                None
                if optimized_anchor_errors.size == 0
                else float(np.median(optimized_anchor_errors))
            ),
            "optimized_svo_anchor_max": (
                None
                if optimized_anchor_errors.size == 0
                else float(np.max(optimized_anchor_errors))
            ),
            "chunks": [
                {
                    "index": idx,
                    "label": chunk.label,
                    "start": chunk.start,
                    "end": chunk.end,
                    "da3_npz": str(chunk.da3_npz),
                    "ply": str(chunk.ply),
                    "initial_transform_to_root": transform_to_json(initial_transforms[idx]),
                    "optimized_transform_to_root": transform_to_json(optimized_transforms[idx]),
                }
                for idx, chunk in enumerate(chunks)
            ],
            "edges_initial": initial_edge_rows,
            "edges_optimized": optimized_edge_rows,
            "svo_anchors_initial": initial_anchor_rows,
            "svo_anchors_optimized": optimized_anchor_rows,
            "point_count": int(merged_points.shape[0]),
        }
        report_path = Path(args.report_json)
        report_path.parent.mkdir(parents=True, exist_ok=True)
        with report_path.open("w") as f:
            json.dump(report, f, indent=2)
            f.write("\n")
        print(f"Saved report: {report_path}")


if __name__ == "__main__":
    main()
