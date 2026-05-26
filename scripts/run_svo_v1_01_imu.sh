#!/usr/bin/env bash
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
OUT_DIR="$REPO_ROOT/outputs/svo_v1_01_imu"
mkdir -p "$OUT_DIR"
DOCKER_CMD="${DOCKER_CMD:-sudo docker}"
export MPLCONFIGDIR="${MPLCONFIGDIR:-/tmp/matplotlib}"

BAG="/workspace/project/data_local/euroc/vicon_room1/V1_01_easy/V1_01_easy.bag"
LAUNCH="/workspace/project/configs/svo_launch/euroc_mono_no_rviz.launch"
POSE_BAG="/workspace/project/outputs/svo_v1_01_imu/svo_pose_cam_imu.bag"

$DOCKER_CMD run --rm --network host \
  -e ROS_MASTER_URI=http://localhost:11311 \
  -e ROS_HOSTNAME=localhost \
  -v "$REPO_ROOT":/workspace/project \
  svo-noetic-base \
  bash -lc "
    set -euo pipefail
    source /opt/ros/noetic/setup.bash
    source /workspace/project/svo_ws/devel/setup.bash

    rm -f '$POSE_BAG' '$POSE_BAG.active'

    roscore > /tmp/svo_v1_01_imu_roscore.log 2>&1 &
    ROSCORE_PID=\$!
    sleep 3

    roslaunch '$LAUNCH' > /workspace/project/outputs/svo_v1_01_imu/svo_node.log 2>&1 &
    SVO_PID=\$!
    sleep 5

    rosbag record -O '$POSE_BAG' /svo/pose_cam/0 > /workspace/project/outputs/svo_v1_01_imu/rosbag_record.log 2>&1 &
    RECORD_PID=\$!
    sleep 2

    rosbag play --clock '$BAG' > /workspace/project/outputs/svo_v1_01_imu/rosbag_play.log 2>&1

    sleep 5
    if kill -0 \$RECORD_PID 2>/dev/null; then
      kill -INT \$RECORD_PID
      wait \$RECORD_PID || true
    fi
    sleep 2
    if [ -f '$POSE_BAG.active' ]; then
      echo 'ERROR: rosbag record did not close cleanly; active bag remains.' >&2
      exit 3
    fi
    if kill -0 \$SVO_PID 2>/dev/null; then
      kill -INT \$SVO_PID || true
    fi
    if kill -0 \$ROSCORE_PID 2>/dev/null; then
      kill -INT \$ROSCORE_PID || true
    fi
    wait \$SVO_PID || true
    wait \$ROSCORE_PID || true
  "

$DOCKER_CMD run --rm --network host \
  -v "$REPO_ROOT":/workspace/project \
  svo-noetic-base \
  bash -lc "
    source /opt/ros/noetic/setup.bash
    python3 /workspace/project/scripts/rosbag_pose_to_tum.py \
      --bag /workspace/project/outputs/svo_v1_01_imu/svo_pose_cam_imu.bag \
      --topic /svo/pose_cam/0 \
      --output /workspace/project/outputs/svo_v1_01_imu/svo_pose_cam_imu_tum.txt
  "

python3 "$REPO_ROOT/scripts/euroc_gt_to_tum.py" \
  --input_csv "$REPO_ROOT/data_local/euroc/vicon_room1/V1_01_easy/V1_01_easy/mav0/state_groundtruth_estimate0/data.csv" \
  --sensor_yaml "$REPO_ROOT/data_local/euroc/vicon_room1/V1_01_easy/V1_01_easy/mav0/cam0/sensor.yaml" \
  --output_tum "$OUT_DIR/gt_cam0_tum.txt"

python3 "$REPO_ROOT/scripts/compare_trajectory_to_gt.py" \
  --estimate "$OUT_DIR/svo_pose_cam_imu_tum.txt" \
  --groundtruth "$OUT_DIR/gt_cam0_tum.txt" \
  --max_dt 0.01 \
  --report_json "$OUT_DIR/svo_pose_cam_imu_vs_gt_report.json" \
  --aligned_tum "$OUT_DIR/svo_pose_cam_imu_aligned_to_gt_tum.txt" \
  --plot_png "$OUT_DIR/svo_pose_cam_imu_vs_gt.png"

echo "SVO V1_01 IMU run complete:"
echo "  $OUT_DIR/svo_pose_cam_imu.bag"
echo "  $OUT_DIR/svo_pose_cam_imu_tum.txt"
echo "  $OUT_DIR/svo_pose_cam_imu_vs_gt_report.json"
echo "  $OUT_DIR/svo_pose_cam_imu_vs_gt.png"
