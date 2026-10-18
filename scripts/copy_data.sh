#!/usr/bin/env bash
# Refresh FlightLab telemetry and recordings from the Talon simulator.
# From the repository root: uv run python python/sims/run_all.py
# (regenerates plots/json/*.json including validation.json, and
# docs/videos/*.mp4), then from the repository root:
#   scripts/copy_data.sh
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(dirname "$SCRIPT_DIR")"
JSON_DIR="$ROOT_DIR/plots/json"
VIDEO_DIR="$ROOT_DIR/docs/videos"
DATA_DEST="$ROOT_DIR/public/data"
VIDEO_DEST="$ROOT_DIR/public/videos"

SCENARIOS=(gps_outage wind emergency obstacle_field)
VIDEOS=(waypoints gps_outage wind emergency obstacle_field)

mkdir -p "$DATA_DEST"
for name in "${SCENARIOS[@]}"; do
  cp "$JSON_DIR/$name.json" "$DATA_DEST/$name.json"
  echo "copied $name.json"
done

cp "$JSON_DIR/validation.json" "$DATA_DEST/validation.json"
echo "copied validation.json"

# Live in-browser run configs (Phase 4): regenerated from the Python scenario
# definitions so the WASM run can never drift from the recorder.
(cd "$ROOT_DIR" && uv run python python/sims/export_live_configs.py)

mkdir -p "$VIDEO_DEST"
for name in "${VIDEOS[@]}"; do
  if [ -f "$VIDEO_DIR/$name.mp4" ]; then
    cp "$VIDEO_DIR/$name.mp4" "$VIDEO_DEST/$name.mp4"
    echo "copied $name.mp4"
  else
    echo "skipping $name.mp4 (not present)"
  fi
done

echo "Telemetry and recordings refreshed into public/data/ and public/videos/"
