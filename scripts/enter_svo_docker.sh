#!/usr/bin/env bash
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

sudo docker run --rm -it \
  -v "$REPO_ROOT":/workspace/project \
  svo-noetic-base \
  bash
