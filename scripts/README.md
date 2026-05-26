# Script Index

This folder now contains only small utilities for individual SVO/DA3 testing and
dataset preparation. Old alignment, overlap, merge, ICP, and graph scripts were
removed from the active repo after the clean restart.

## Data Prep

- `extract_rosbag_images.py`: extract image frames from a ROS bag.
- `sync_frames_to_svo.py`: match extracted image timestamps to SVO trajectory poses.
- `prepare_tum_rgbd_chunks.py`: associate TUM RGB-D RGB/depth/GT data and export DA3-ready chunks.

## SVO Trajectory

- `rosbag_pose_to_tum.py`: convert ROS pose messages to TUM trajectory text format.
- `euroc_gt_to_tum.py`: convert EuRoC ground-truth CSV to TUM, optionally transformed into a sensor frame.
- `compare_trajectory_to_gt.py`: align an estimated TUM trajectory to ground truth with Sim(3) and report ATE-style errors.
- `run_svo_v1_01_imu.sh`: run the V1_01_easy SVO/VIO trajectory baseline, export TUM, and compare to cam0-frame GT.

## DA3 Inspection

- `da3_npz_to_ply.py`: convert DA3 `mini_npz` output into a simple colored PLY.
- `make_one_frame_ply.py`: build a diagnostic PLY for a single frame.

## Environment Helpers

- `setup_svo_ws.sh`: prepare the local SVO catkin workspace.
- `enter_svo_docker.sh`: enter the SVO Docker environment.
