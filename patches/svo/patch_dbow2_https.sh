#!/usr/bin/env bash
set -euo pipefail

DBOW2_DIR="${1:-svo_ws/src/dbow2_catkin}"

grep -RIl "git@github.com:dorian3d/DBoW2.git" "$DBOW2_DIR" \
  | xargs sed -i 's#git@github.com:dorian3d/DBoW2.git#https://github.com/dorian3d/DBoW2.git#g'

echo "Patched dbow2_catkin DBoW2 clone URL to HTTPS."
