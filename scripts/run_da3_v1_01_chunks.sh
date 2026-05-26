#!/usr/bin/env bash
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$REPO_ROOT"

OUTPUT_ROOT="${OUTPUT_ROOT:-outputs/da3_v1_01_16f}"
CHUNK_SIZE="${CHUNK_SIZE:-16}"
OVERLAP="${OVERLAP:-4}"
MAX_CHUNKS="${MAX_CHUNKS:-40}"
PROCESS_RES="${PROCESS_RES:-384}"
EXPORT_FORMAT="${EXPORT_FORMAT:-mini_npz}"

python3 scripts/prepare_euroc_da3_chunks.py \
  --output_root "$OUTPUT_ROOT" \
  --chunk_size "$CHUNK_SIZE" \
  --overlap "$OVERLAP" \
  --max_chunks "$MAX_CHUNKS"

python3 - <<'PY' "$OUTPUT_ROOT" "$MAX_CHUNKS" "$PROCESS_RES" "$EXPORT_FORMAT"
import json
import subprocess
import sys
from pathlib import Path

output_root = Path(sys.argv[1])
max_chunks = int(sys.argv[2])
process_res = sys.argv[3]
export_format = sys.argv[4]
manifest = json.loads((output_root / "chunks_manifest.json").read_text())

for chunk in manifest["chunks"][:max_chunks]:
    name = chunk["name"]
    images = chunk["images"]
    export_dir = output_root / "da3" / name
    print(f"\n=== DA3 {name} ===", flush=True)
    cmd = [
        "conda", "run", "-n", "da3", "da3", "images", images,
        "--export-dir", str(export_dir),
        "--export-format", export_format,
        "--device", "cuda",
        "--process-res", process_res,
        "--auto-cleanup",
    ]
    subprocess.run(cmd, check=True)
PY

echo "DA3 chunk batch complete:"
echo "  $OUTPUT_ROOT/chunks_manifest.json"
echo "  $OUTPUT_ROOT/da3/"
