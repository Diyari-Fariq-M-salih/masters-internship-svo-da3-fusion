# SVO + Depth Anything 3 Fusion Pipeline

Prototype repository for a Master's internship project on dense 3D reconstruction from RGB image sequences using **SVO** and **Depth Anything 3**.

The goal is to build a practical, modular pipeline that combines:

- **SVO** for camera trajectory / pose estimation
- **Depth Anything 3** for dense monocular depth prediction
- **Camera intrinsics** for depth back-projection
- **Point-cloud chunking** for local reconstruction blocks
- **Sim(3) alignment** for monocular scale ambiguity

The target outputs are:

- `.ply` point-cloud chunks and merged maps
- `.txt` trajectory files, preferably in TUM format
- experiment summaries for report writing and reproducibility

---

## 1. Project Motivation

This project investigates whether a lightweight hybrid reconstruction pipeline can run under constrained laptop hardware.

Previous experiments showed:

- **SVO** is efficient and useful for camera motion estimation, but can fail on difficult sequences and suffers from monocular scale ambiguity.
- **Depth Anything 3** provides stable dense depth maps from RGB images and is practical on the available RTX 2060 6 GB GPU.
- **LiteVGGT / other transformer-heavy methods** were not practical on the current hardware because of GPU memory limitations.

Therefore, the current direction is to combine SVO and Depth Anything 3 instead of relying on a large end-to-end reconstruction model.

---

## 2. Current Pipeline Idea

```text
RGB video / image sequence
        |
        |--------------------------|
        |                          |
       SVO                 Depth Anything 3
        |                          |
 camera poses / trajectory      dense depth maps
        |                          |
        |------ camera intrinsics -|
                     |
             back-project to 3D
                     |
          local point-cloud chunks
                     |
       optional Sim(3) alignment
                     |
      global .ply map + .txt trajectory
```

The first implementation should be offline and modular. SVO and Depth Anything 3 can be run separately, then their outputs can be fused by Python scripts.

---

## 3. First Milestones

### Milestone 1  One-frame prototype

Generate one valid colored `.ply` file from:

```text
one RGB image
+ one DA3 depth map
+ camera intrinsics
+ one SVO/camera pose
= one colored point cloud
```

This is the most important first sanity check.

### Milestone 2 First local chunk

Generate:

```text
PCL1 = frames/keyframes 1–16
```

using SVO poses and DA3 depth maps.

### Milestone 3 100-frame sequence

Generate multiple point-cloud chunks:

```text
PCL1 = 1–16
PCL2 = 17–32
PCL3 = 33–48
...
```

or, if overlap is needed later:

```text
PCL1 = 1–16
PCL2 = 16–32
PCL3 = 32–48
...
```

### Milestone 4 Sim(3) alignment

Add Sim(3) alignment for trajectory evaluation and point-cloud merging.

This is needed because monocular SVO and monocular DA3 depth are not guaranteed to have metric scale.

---

## 4. Recommended Repository Structure

```text
svo-da3-fusion-pipeline/
├── README.md
├── CHANGELOG.md
├── TODO.md
├── requirements.txt
├── environment.yml
├── .gitignore
│
├── configs/
│   ├── camera_euroc.yaml
│   ├── camera_bonn.yaml
│   └── pipeline_default.yaml
│
├── scripts/
│   ├── extract_frames.py
│   ├── run_da3_depth.py
│   ├── export_svo_traj.py
│   ├── sync_frames_poses.py
│   ├── make_pcl_chunks.py
│   ├── merge_ply.py
│   ├── align_sim3.py
│   └── evaluate_outputs.py
│
├── src/
│   └── svo_da3_fusion/
│       ├── __init__.py
│       ├── io_utils.py
│       ├── geometry.py
│       ├── trajectory.py
│       ├── pointcloud.py
│       ├── sim3.py
│       └── visualization.py
│
├── notebooks/
│   ├── 01_check_depth_maps.ipynb
│   ├── 02_visualize_trajectory.ipynb
│   └── 03_pointcloud_sanity_check.ipynb
│
├── docs/
│   ├── handover_summary.md
│   ├── supervisor_sketch_interpretation.md
│   ├── pipeline_design.md
│   ├── experiment_log.md
│   └── known_issues.md
│
├── experiments/
│   ├── sequence_001_easy/
│   │   ├── README.md
│   │   ├── config.yaml
│   │   └── run_summary.md
│   └── sequence_002_medium/
│       ├── README.md
│       ├── config.yaml
│       └── run_summary.md
│
└── examples/
    ├── intrinsics_example.yaml
    ├── trajectory_example.txt
    └── sync_table_example.csv
```

Large data should live outside the Git repository, for example:

```text
/data/svo_da3/
├── sequence_001/
│   ├── rgb/
│   ├── depth_da3/
│   ├── svo/
│   └── outputs/
```

Only small examples, configs, scripts, and documentation should be committed.

---

## 5. Input and Output Formats

### RGB frames

Recommended naming:

```text
frame_000000.png
frame_000001.png
frame_000002.png
...
```

### Camera intrinsics

Example `intrinsics.yaml`:

```yaml
camera:
  width: 640
  height: 480
  fx: 525.0
  fy: 525.0
  cx: 319.5
  cy: 239.5
  distortion_model: none
```

If images are resized, intrinsics must also be resized:

```text
fx_new = fx_old * new_width  / old_width
fy_new = fy_old * new_height / old_height
cx_new = cx_old * new_width  / old_width
cy_new = cy_old * new_height / old_height
```

### DA3 depth maps

Use raw numerical depth arrays:

```text
depth_da3/frame_000000.npy
depth_da3/frame_000001.npy
```

Do **not** use color heatmap PNGs for geometry. PNG visualizations are useful only for inspection.

### SVO trajectory

Preferred trajectory format is TUM:

```text
# timestamp tx ty tz qx qy qz qw
0.000000 0.0000 0.0000 0.0000 0.0000 0.0000 0.0000 1.0000
0.033333 0.0123 0.0001 0.0020 0.0010 0.0000 0.0005 0.9999
```

### Point-cloud outputs

Recommended output names:

```text
outputs/pcl_chunks/pcl_000001_000016.ply
outputs/pcl_chunks/pcl_000017_000032.ply
outputs/global/merged_raw.ply
outputs/global/merged_sim3.ply
```

---

## 6. Geometry: Depth Back-projection

Given pixel coordinates `(u, v)`, depth `z`, and camera intrinsics:

```text
x_c = (u - cx) * z / fx
y_c = (v - cy) * z / fy
z_c = z
```

This gives a 3D point in the camera coordinate system:

```text
X_c = [x_c, y_c, z_c]^T
```

Given a camera-to-world pose `T_w_c`, transform it into the world frame:

```text
X_w = T_w_c * X_c
```

Each point should store:

```text
x, y, z, red, green, blue
```

Implementation notes:

- ignore invalid depth values
- ignore depth values that are too small or too large
- use a pixel stride such as 2, 4, or 8 to reduce point count
- test one frame before testing a full chunk
- verify whether the SVO pose is `T_w_c` or `T_c_w`

---

## 7. Suggested Script Responsibilities

| Script | Purpose | Priority |
|---|---|---|
| `extract_frames.py` | Extract a clean RGB frame sequence from video/dataset. | High |
| `run_da3_depth.py` | Run DA3 and save raw depth maps as `.npy`. | High |
| `export_svo_traj.py` | Convert SVO ROS pose output to TUM `.txt`. | High |
| `sync_frames_poses.py` | Match RGB frames, DA3 depths, and SVO poses. | High |
| `make_pcl_chunks.py` | Back-project depth and poses into PLY chunks. | Highest |
| `merge_ply.py` | Merge local PLY chunks into one cloud. | Medium |
| `align_sim3.py` | Compute/apply Sim(3) alignment. | Medium |
| `evaluate_outputs.py` | Summarize trajectory, point count, and sanity checks. | Medium |

---

## 8. Example Command Flow, inspection at each stage
## Separate commands = safer development and easier debugging.

```bash
# 1. Extract or prepare frames
python scripts/extract_frames.py \
  --input data_raw/video.mp4 \
  --output /data/svo_da3/sequence_001/rgb \
  --num_frames 100

# 2. Run DA3 depth prediction
python scripts/run_da3_depth.py \
  --rgb_dir /data/svo_da3/sequence_001/rgb \
  --output_dir /data/svo_da3/sequence_001/depth_da3

# 3. Export or prepare SVO trajectory
python scripts/export_svo_traj.py \
  --input_bag /data/svo_da3/sequence_001/svo/poses.bag \
  --output /data/svo_da3/sequence_001/svo/svo_traj_raw.txt

# 4. Synchronize frames, depths, and poses
python scripts/sync_frames_poses.py \
  --rgb_dir /data/svo_da3/sequence_001/rgb \
  --depth_dir /data/svo_da3/sequence_001/depth_da3 \
  --trajectory /data/svo_da3/sequence_001/svo/svo_traj_raw.txt \
  --output /data/svo_da3/sequence_001/sync_table.csv

# 5. Generate point-cloud chunks
python scripts/make_pcl_chunks.py \
  --sync_table /data/svo_da3/sequence_001/sync_table.csv \
  --intrinsics /data/svo_da3/sequence_001/intrinsics.yaml \
  --chunk_size 16 \
  --stride 4 \
  --output_dir /data/svo_da3/sequence_001/outputs/pcl_chunks
```

---

## 9. Critical Risks

| Risk | Symptom | Mitigation |
|---|---|---|
| Frame/depth/pose mismatch | Point cloud looks exploded or smeared. | Use a synchronization table. |
| Wrong intrinsics after resizing | Geometry looks distorted. | Scale `fx`, `fy`, `cx`, `cy`. |
| Wrong pose convention | Cloud appears behind camera, flipped, or rotated. | Check `T_w_c` vs `T_c_w`; test one frame. |
| Unknown scale | Cloud or trajectory is too large/small. | Use Sim(3), ground truth, or RGB-D later. |
| DA3 relative depth | Shape is plausible but not metric. | Treat as relative depth initially. |
| SVO tracking failure | Jumps, invalid poses, trajectory explosion. | Detect pose jumps and discard/split bad chunks. |
| Huge PLY files | Slow visualization or memory issues. | Use stride, max-depth filtering, voxel downsampling. |
| Dynamic objects | Ghosting or duplicated moving objects. | Start with static/easy data; mask later if needed. |

---

## 10. Sim(3) vs SE(3)

For monocular RGB-only reconstruction, scale is ambiguous. Therefore, Sim(3) alignment is usually required.

```text
Sim(3) = scale + rotation + translation
SE(3)  = rotation + translation
```

Use **Sim(3)** when:

- SVO is monocular
- DA3 depth is relative
- trajectory or chunks have unknown scale

Use **SE(3)** when:

- metric depth is available
- RGB-D is used
- VIO/IMU provides metric scale
- scale is already known and fixed

For the first prototype, do not overcomplicate alignment. Generate raw chunks first, inspect them visually, then add Sim(3).

---

## 11. Experiment Logging Template

Each experiment should have a `run_summary.md`.

```markdown
# Run Summary: sequence_001_easy

## Date
YYYY-MM-DD

## Input
- Frames: 100
- Resolution: 640x480
- Intrinsics: intrinsics.yaml
- Depth source: Depth Anything 3
- Pose source: SVO monocular

## Command
```bash
python scripts/make_pcl_chunks.py --config experiments/sequence_001_easy/config.yaml
```

## Outputs
- `pcl_000001_000016.ply`
- `pcl_000017_000032.ply`
- `trajectory_raw.txt`

## Observations
- One-frame cloud:
- PCL1 visual quality:
- Scale issue:
- Pose/depth synchronization issue:
- Coordinate convention issue:

## Status
Pass / Fail / Needs debugging
```

---

## 12. GitHub Issue Plan

Recommended first issues:

```text
[DOC] Add handover summary and supervisor sketch interpretation
[DATA] Prepare 100-frame easy sequence with intrinsics
[DA3] Export raw depth maps as .npy
[SVO] Export trajectory in TUM format
[SYNC] Match RGB frames, DA3 depths, and SVO poses
[PCL] Generate one-frame colored .ply
[PCL] Generate PCL1 from 16 frames/keyframes
[PCL] Generate full 100-frame chunked pipeline
[SIM3] Implement trajectory Sim(3) alignment
[MERGE] Merge chunk point clouds into global map
[EVAL] Add visual sanity checks and run summaries
```

Suggested labels:

```text
data
svo
depth-anything-3
pointcloud
trajectory
sim3
bug
experiment
documentation
priority-high
blocked
```

Suggested GitHub Project columns:

```text
Backlog
Ready
In Progress
Blocked
Needs Visual Check
Done
```

---

## 13. Branch Strategy

Keep the workflow simple:

```text
main      = stable, documented results
dev       = current working integration branch
feature/* = individual tasks
```

Example branches:

```text
feature/one-frame-ply
feature/da3-depth-export
feature/svo-trajectory-export
feature/sync-table
feature/pcl-chunks
feature/sim3-alignment
```

Example commit messages:

```text
Add one-frame depth backprojection script
Fix intrinsics scaling after image resize
Add TUM trajectory parser
Save PCL chunks with RGB colors
Document Sim3 vs SE3 alignment choice
```

---

## 14. Files Not to Commit

Do not commit large generated or external files:

```text
large datasets
rosbags
Depth Anything model weights
full depth map folders
full .ply result folders
video files
large screenshots
temporary outputs
```

Use external storage for these files and record paths in experiment summaries.

Example:

```text
Data location:
/media/diyari/SSD/svo_da3/sequence_001/
```
---

## 16. Current Status

The project is ready to move from handover/design into implementation.

Immediate next step:

```text
Implement make_pcl_chunks.py for one frame only.
```

After one frame works:

```text
1 frame → 2 frames → 16-frame PCL1 → 100-frame chunked sequence → Sim(3) alignment
```
Final target: parallel/asynchronous, not purely series.

For the offline prototype, we may run them separately or in series for debugging:

Run SVO → save trajectory.txt
Run DA3 → save depth maps
Fuse outputs → save .ply

But for the real-time drone version, they should run like this:
```text
Camera stream
   ↓
Frame buffer
   ├── SVO thread: runs on every frame, real-time pose tracking
   └── DA3 thread: runs on selected keyframes, slower dense depth
                 ↓
Fusion thread: waits until it has pose + depth for a keyframe
                 ↓
Point-cloud update
```