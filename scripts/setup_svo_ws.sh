#!/usr/bin/env bash
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SVO_WS="$REPO_ROOT/svo_ws"

mkdir -p "$SVO_WS/src"

if [ ! -d "$SVO_WS/src/rpg_svo_pro_open" ]; then
  cp -r "$REPO_ROOT/external/rpg_svo_pro_open" "$SVO_WS/src/"
fi

cp "$REPO_ROOT/patches/svo/dependencies_https.yaml" "$SVO_WS/src/dependencies_https.yaml"

cd "$SVO_WS"

catkin config --init --mkdirs --extend /opt/ros/noetic \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DEIGEN3_INCLUDE_DIR=/usr/include/eigen3

cd "$SVO_WS/src"

vcs-import < dependencies_https.yaml

touch minkindr/minkindr_python/CATKIN_IGNORE

if [ -f "$REPO_ROOT/patches/svo/patch_dbow2_https.sh" ]; then
  "$REPO_ROOT/patches/svo/patch_dbow2_https.sh" "$SVO_WS/src/dbow2_catkin"
fi

echo "SVO workspace prepared at: $SVO_WS"
echo "Next: run catkin build inside the Docker container."