#!/usr/bin/env bash
set -euo pipefail

# Usage:
#   bash scripts/batch_da3_chunks_to_ply.sh
#
# Optional overrides:
#   OUTPUT_ROOT=outputs/da3_v1_01_16f bash scripts/batch_da3_chunks_to_ply.sh
#   STRIDE=2 bash scripts/batch_da3_chunks_to_ply.sh
#   EXTRINSIC_MODE=as_t_w_c bash scripts/batch_da3_chunks_to_ply.sh

REPO_ROOT="${REPO_ROOT:-$(pwd)}"
OUTPUT_ROOT="${OUTPUT_ROOT:-outputs/da3_v1_01_16f}"

STRIDE="${STRIDE:-2}"
MIN_DEPTH="${MIN_DEPTH:-0.2}"
MAX_DEPTH="${MAX_DEPTH:-8.0}"
CONF_PERCENTILE="${CONF_PERCENTILE:-25}"
EXTRINSIC_MODE="${EXTRINSIC_MODE:-invert}"

CONVERTER="${CONVERTER:-scripts/da3_chunk_npz_to_ply.py}"

CHUNKS_DIR="$OUTPUT_ROOT/chunks"
DA3_DIR="$OUTPUT_ROOT/da3"
PLY_DIR="$OUTPUT_ROOT/ply"

mkdir -p "$PLY_DIR"

echo "Repo root:       $REPO_ROOT"
echo "Output root:     $OUTPUT_ROOT"
echo "Chunks dir:      $CHUNKS_DIR"
echo "DA3 dir:         $DA3_DIR"
echo "PLY dir:         $PLY_DIR"
echo "Converter:       $CONVERTER"
echo "Stride:          $STRIDE"
echo "Depth range:     $MIN_DEPTH to $MAX_DEPTH"
echo "Conf percentile: $CONF_PERCENTILE"
echo "Extrinsic mode:  $EXTRINSIC_MODE"
echo

if [[ ! -f "$CONVERTER" ]]; then
  echo "[ERROR] Converter script not found: $CONVERTER"
  echo "Expected your Python converter at scripts/da3_mini_npz_to_ply.py"
  exit 1
fi

if [[ ! -d "$CHUNKS_DIR" ]]; then
  echo "[ERROR] Missing chunks dir: $CHUNKS_DIR"
  exit 1
fi

if [[ ! -d "$DA3_DIR" ]]; then
  echo "[ERROR] Missing DA3 dir: $DA3_DIR"
  exit 1
fi

count_ok=0
count_skip=0
count_fail=0

for chunk_path in "$CHUNKS_DIR"/chunk_*; do
  [[ -d "$chunk_path" ]] || continue

  chunk_name="$(basename "$chunk_path")"

  frames_csv="$chunk_path/frames.csv"
  image_dir="$chunk_path/images"
  da3_npz="$DA3_DIR/$chunk_name/exports/mini_npz/results.npz"
  output_ply="$PLY_DIR/${chunk_name}.ply"

  echo "============================================================"
  echo "Chunk: $chunk_name"

  if [[ ! -f "$frames_csv" ]]; then
    echo "[SKIP] Missing frames.csv: $frames_csv"
    count_skip=$((count_skip + 1))
    continue
  fi

  if [[ ! -d "$image_dir" ]]; then
    echo "[SKIP] Missing images dir: $image_dir"
    count_skip=$((count_skip + 1))
    continue
  fi

  if [[ ! -f "$da3_npz" ]]; then
    echo "[SKIP] Missing DA3 npz: $da3_npz"
    count_skip=$((count_skip + 1))
    continue
  fi

  echo "NPZ:    $da3_npz"
  echo "CSV:    $frames_csv"
  echo "Images: $image_dir"
  echo "PLY:    $output_ply"

  if python3 "$CONVERTER" \
      --da3_npz "$da3_npz" \
      --frames_csv "$frames_csv" \
      --image_dir "$image_dir" \
      --output "$output_ply" \
      --stride "$STRIDE" \
      --min_depth "$MIN_DEPTH" \
      --max_depth "$MAX_DEPTH" \
      --conf_percentile "$CONF_PERCENTILE" \
      --extrinsic_mode "$EXTRINSIC_MODE"; then

    echo "[OK] Saved: $output_ply"
    count_ok=$((count_ok + 1))
  else
    echo "[FAIL] Conversion failed for $chunk_name"
    count_fail=$((count_fail + 1))
  fi

  echo
done

echo "============================================================"
echo "Batch PLY conversion finished."
echo "Successful: $count_ok"
echo "Skipped:    $count_skip"
echo "Failed:     $count_fail"
echo "PLY output: $PLY_DIR"