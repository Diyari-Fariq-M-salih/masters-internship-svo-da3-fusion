#!/usr/bin/env bash
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

sudo docker run --rm -it \
  --network host \
  -e ROS_MASTER_URI=http://localhost:11311 \
  -e ROS_HOSTNAME=localhost \
  -v "$REPO_ROOT":/workspace/project \
  svo-noetic-base \
  bash