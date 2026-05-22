#!/usr/bin/env python3
"""
Chain DA3-inverted overlap alignments into one merged PLY.

Each input chunk keeps two roles:
- DA3 NPZ + frame range: used to estimate the overlap Sim(3) to the previous
  chunk from DA3-inverted camera centers.
- PLY: points to transform into the first chunk's coordinate frame.

This is different from running pairwise alignment repeatedly: transforms are
composed so every later chunk lands in the first chunk's frame.
"""

import argparse
import json
from dataclasses import dataclass
from pathlib import Path

import numpy as np

from align_da3_overlap_ply import (
    da3_inv_poses,
    estimate_rotation_from_orientations,
    estimate_rotation_hybrid,
    estimate_scale_translation_with_fixed_rotation,
    estimate_sim3_umeyama,
    estimate_translation_with_fixed_rotation_scale,
    load_ascii_ply,
    rotation_errors_degrees,
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


def estimate_source_to_target(args, source, target, overlap_start, overlap_end):
    frame_ids, source_centers, source_rotations = da3_inv_poses(
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
    if not np.array_equal(frame_ids, target_frame_ids):
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

    return {
        "frame_ids": frame_ids,
        "scale": scale,
        "rotation": rotation,
        "translation": translation,
        "errors": errors,
        "rotation_errors": rotation_errors,
    }


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


def transform_to_json(transform):
    scale, rotation, translation = transform
    return {
        "scale": float(scale),
        "rotation": rotation.tolist(),
        "translation": translation.tolist(),
    }


def error_stats(errors):
    return {
        "rmse": float(np.sqrt(np.mean(errors * errors))),
        "median": float(np.median(errors)),
        "mean": float(np.mean(errors)),
        "max": float(np.max(errors)),
    }


def main():
    parser = argparse.ArgumentParser(
        description="Merge DA3-inverted chunks by chaining overlap Sim(3)s."
    )
    parser.add_argument(
        "--chunk",
        action="append",
        type=parse_chunk,
        required=True,
        help="Chunk as START:END:DA3_NPZ:PLY. Repeat in chain order.",
    )
    parser.add_argument("--output", required=True)
    parser.add_argument("--report_json", default=None)
    parser.add_argument("--save_transformed_dir", default=None)
    parser.add_argument(
        "--rotation_mode",
        choices=["centers", "orientations", "hybrid"],
        default="centers",
    )
    parser.add_argument("--orientation_weight", type=float, default=1.0)
    parser.add_argument(
        "--scale_override",
        type=float,
        default=None,
        help="Optional fixed relative scale for every pairwise overlap.",
    )
    args = parser.parse_args()

    chunks = args.chunk
    if len(chunks) < 2:
        raise ValueError("At least two chunks are required.")

    transformed_dir = Path(args.save_transformed_dir) if args.save_transformed_dir else None
    if transformed_dir is not None:
        transformed_dir.mkdir(parents=True, exist_ok=True)

    global_transforms = [(1.0, np.eye(3), np.zeros(3))]
    steps = []

    all_points = []
    all_colors = []

    for idx, chunk in enumerate(chunks):
        if idx > 0:
            target = chunks[idx - 1]
            overlap_start = max(target.start, chunk.start)
            overlap_end = min(target.end, chunk.end)
            if overlap_end - overlap_start + 1 < 3:
                raise ValueError(
                    f"Need at least 3 overlap frames between {target.label} "
                    f"and {chunk.label}, got {overlap_start}-{overlap_end}."
                )

            rel = estimate_source_to_target(
                args,
                source=chunk,
                target=target,
                overlap_start=overlap_start,
                overlap_end=overlap_end,
            )
            relative_transform = (
                rel["scale"],
                rel["rotation"],
                rel["translation"],
            )
            global_transforms.append(compose(global_transforms[idx - 1], relative_transform))

            steps.append({
                "source": chunk.label,
                "target": target.label,
                "overlap_start_frame": overlap_start,
                "overlap_end_frame": overlap_end,
                "overlap_frame_ids": rel["frame_ids"].tolist(),
                "relative_transform_source_to_target": transform_to_json(relative_transform),
                "global_transform_source_to_root": transform_to_json(global_transforms[idx]),
                "alignment_error": error_stats(rel["errors"]),
                "rotation_error_degrees": error_stats(rel["rotation_errors"]),
            })

            print(
                f"{chunk.label} -> {target.label} overlap "
                f"{overlap_start}-{overlap_end}: "
                f"scale={rel['scale']:.9f}, "
                f"rmse={np.sqrt(np.mean(rel['errors'] * rel['errors'])):.9f}, "
                f"rot_rmse={np.sqrt(np.mean(rel['rotation_errors'] * rel['rotation_errors'])):.6f} deg"
            )
        else:
            print(f"{chunk.label}: root frame")

        points, colors = load_ascii_ply(chunk.ply)
        transformed_points = apply_transform(points, global_transforms[idx])
        all_points.append(transformed_points)
        all_colors.append(colors)

        if transformed_dir is not None:
            transformed_path = transformed_dir / f"pcl_{chunk.label}_global.ply"
            save_ascii_ply(transformed_path, transformed_points, colors)
            print(f"Saved transformed chunk: {transformed_path}")

    merged_points = np.concatenate(all_points, axis=0)
    merged_colors = np.concatenate(all_colors, axis=0)
    save_ascii_ply(args.output, merged_points, merged_colors)

    print(f"Saved merged cloud: {args.output}")
    print(f"Total points: {merged_points.shape[0]}")

    if args.report_json is not None:
        report = {
            "output": args.output,
            "rotation_mode": args.rotation_mode,
            "orientation_weight": args.orientation_weight,
            "scale_override": args.scale_override,
            "chunks": [
                {
                    "label": chunk.label,
                    "start": chunk.start,
                    "end": chunk.end,
                    "da3_npz": str(chunk.da3_npz),
                    "ply": str(chunk.ply),
                    "global_transform_to_root": transform_to_json(global_transforms[idx]),
                }
                for idx, chunk in enumerate(chunks)
            ],
            "steps": steps,
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
