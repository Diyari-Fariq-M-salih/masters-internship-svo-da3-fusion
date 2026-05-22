# Script Index

This folder contains lightweight project utilities for SVO, DA3, and fusion
experiments. Most scripts are intended to be run from the repository root.

## Data Prep

- `extract_rosbag_images.py`: extract image frames from a ROS bag.
- `sync_frames_to_svo.py`: match extracted image timestamps to SVO trajectory poses.
- `prepare_tum_rgbd_chunks.py`: associate TUM RGB-D RGB/depth/GT data and export DA3-ready chunks.

## Trajectory Tools

- `rosbag_pose_to_tum.py`: convert ROS pose messages to TUM trajectory text format.
- `compare_trajectory_to_gt.py`: align an estimated TUM trajectory to ground truth with Sim(3) and report ATE-style errors.
- `compare_svo_da3_centers.py`: compare SVO camera centers against DA3-inverted camera centers.

## Point Cloud Generation

- `da3_npz_to_ply.py`: convert DA3 `mini_npz` output into a simple colored PLY.
- `make_one_frame_ply.py`: build a diagnostic PLY for a single frame.
- `make_pcl_chunk_from_da3_npz.py`: build a multi-frame colored PLY from DA3 depth using identity, DA3, DA3-inverted, SVO, or experimental SVO/DA3 pose modes.

## Alignment And Fusion

- `align_da3_overlap_ply.py`: align a DA3-inverted source chunk to a target chunk using overlap camera centers and optional orientation constraints.
- `align_da3inv_ply_to_svo.py`: align an individual DA3-inverted chunk into the SVO frame.
- `merge_ascii_ply.py`: concatenate simple generated ASCII PLY files.
- `merge_da3_overlap_chain.py`: compose pairwise DA3-overlap Sim(3) transforms into one root-frame merged PLY.
- `refine_da3_overlap_icp.py`: ICP diagnostics for overlap-aligned chunks.

## Diagnostics

- `compare_da3_same_frame.py`: compare the same RGB frame reconstructed in two different DA3 chunk contexts.
- `make_da3_depth_scaled_chunk.py`: rebuild a DA3-inverted chunk after overlap-derived per-frame depth scaling.

## Environment Helpers

- `setup_svo_ws.sh`: prepare the local SVO catkin workspace.
- `enter_svo_docker.sh`: enter the SVO Docker environment.
