#!/usr/bin/env python3
"""Prepare overlapping EuRoC image chunks for DA3 testing."""

from __future__ import annotations

import argparse
import csv
import json
import shutil
from pathlib import Path


def load_cam_csv(path: str | Path) -> list[dict]:
    rows = []
    with Path(path).open(newline="") as f:
        reader = csv.DictReader(f)
        for idx, row in enumerate(reader):
            rows.append(
                {
                    "frame_id": idx,
                    "timestamp_ns": int(row["#timestamp [ns]"]),
                    "timestamp": int(row["#timestamp [ns]"]) * 1e-9,
                    "filename": row["filename"],
                }
            )
    return rows


def chunk_ranges(count: int, chunk_size: int, overlap: int, max_chunks: int | None) -> list[tuple[int, int]]:
    if chunk_size <= 0:
        raise ValueError("chunk_size must be positive")
    if overlap < 0 or overlap >= chunk_size:
        raise ValueError("overlap must be >= 0 and < chunk_size")
    step = chunk_size - overlap
    ranges = []
    start = 0
    while start < count:
        end = min(start + chunk_size - 1, count - 1)
        ranges.append((start, end))
        if end == count - 1:
            break
        if max_chunks is not None and len(ranges) >= max_chunks:
            break
        start += step
    return ranges


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--cam_csv",
        default="data_local/euroc/vicon_room1/V1_01_easy/V1_01_easy/mav0/cam0/data.csv",
    )
    parser.add_argument(
        "--image_dir",
        default="data_local/euroc/vicon_room1/V1_01_easy/V1_01_easy/mav0/cam0/data",
    )
    parser.add_argument("--output_root", default="outputs/da3_v1_01")
    parser.add_argument("--chunk_size", type=int, default=32)
    parser.add_argument("--overlap", type=int, default=8)
    parser.add_argument("--start_frame", type=int, default=0)
    parser.add_argument("--end_frame", type=int, default=None)
    parser.add_argument("--max_chunks", type=int, default=None)
    args = parser.parse_args()

    all_rows = load_cam_csv(args.cam_csv)
    end_frame = len(all_rows) - 1 if args.end_frame is None else args.end_frame
    selected = all_rows[args.start_frame : end_frame + 1]
    if not selected:
        raise ValueError("No frames selected")

    output_root = Path(args.output_root)
    chunks_root = output_root / "chunks"
    chunks_root.mkdir(parents=True, exist_ok=True)

    ranges = chunk_ranges(len(selected), args.chunk_size, args.overlap, args.max_chunks)
    manifest = {
        "source": {
            "cam_csv": args.cam_csv,
            "image_dir": args.image_dir,
            "start_frame": args.start_frame,
            "end_frame": end_frame,
        },
        "chunk_size": args.chunk_size,
        "overlap": args.overlap,
        "chunks": [],
    }

    image_dir = Path(args.image_dir)
    for chunk_idx, (local_start, local_end) in enumerate(ranges):
        rows = selected[local_start : local_end + 1]
        global_start = rows[0]["frame_id"]
        global_end = rows[-1]["frame_id"]
        chunk_name = f"chunk_{chunk_idx:03d}_{global_start:06d}_{global_end:06d}"
        chunk_dir = chunks_root / chunk_name / "images"
        chunk_dir.mkdir(parents=True, exist_ok=True)

        with (chunks_root / chunk_name / "frames.csv").open("w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(["local_id", "frame_id", "timestamp_ns", "timestamp", "source_filename", "filename"])
            for local_id, row in enumerate(rows):
                out_name = f"frame_{row['frame_id']:06d}.png"
                shutil.copy2(image_dir / row["filename"], chunk_dir / out_name)
                writer.writerow(
                    [
                        local_id,
                        row["frame_id"],
                        row["timestamp_ns"],
                        f"{row['timestamp']:.9f}",
                        row["filename"],
                        out_name,
                    ]
                )

        manifest["chunks"].append(
            {
                "chunk_id": chunk_idx,
                "name": chunk_name,
                "path": str(chunks_root / chunk_name),
                "images": str(chunk_dir),
                "frame_start": global_start,
                "frame_end": global_end,
                "frame_count": len(rows),
            }
        )

    with (output_root / "chunks_manifest.json").open("w") as f:
        json.dump(manifest, f, indent=2)
        f.write("\n")

    print(f"Selected frames: {args.start_frame}-{end_frame} ({len(selected)} frames)")
    print(f"Chunks: {len(manifest['chunks'])}")
    print(f"Manifest: {output_root / 'chunks_manifest.json'}")
    for chunk in manifest["chunks"][:5]:
        print(f"  {chunk['name']}: {chunk['frame_count']} frames")
    if len(manifest["chunks"]) > 5:
        print("  ...")


if __name__ == "__main__":
    main()
