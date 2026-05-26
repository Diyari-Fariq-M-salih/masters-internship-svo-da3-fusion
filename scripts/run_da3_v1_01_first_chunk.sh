#!/usr/bin/env bash
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$REPO_ROOT"

python3 scripts/prepare_euroc_da3_chunks.py \
  --output_root outputs/da3_v1_01_16f \
  --chunk_size 16 \
  --overlap 4 \
  --max_chunks 1

CHUNK_DIR="outputs/da3_v1_01_16f/chunks/chunk_000_000000_000015"

conda run -n da3 da3 images "$CHUNK_DIR/images" \
  --export-dir "outputs/da3_v1_01_16f/da3/chunk_000_000000_000015" \
  --export-format mini_npz \
  --device cuda \
  --process-res 384 \
  --auto-cleanup

echo "DA3 first chunk complete:"
echo "  outputs/da3_v1_01_16f/chunks_manifest.json"
echo "  outputs/da3_v1_01_16f/da3/chunk_000_000000_000015"
