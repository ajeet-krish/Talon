#!/usr/bin/env bash
# Refresh replay telemetry and recordings from the Talon simulator.
# From the Talon repo root: uv run python sims/run_all.py
# (regenerates plots/json/*.json including validation.json, and
# docs/videos/*.mp4), then from this app directory:
#   scripts/copy_data.sh
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
APP_DIR="$(dirname "$SCRIPT_DIR")"
REPO_DIR="$(dirname "$APP_DIR")"
JSON_DIR="$REPO_DIR/plots/json"
VIDEO_DIR="$REPO_DIR/docs/videos"
DATA_DEST="$APP_DIR/public/data"
VIDEO_DEST="$APP_DIR/public/videos"

SCENARIOS=(gps_outage wind emergency obstacle_field)
VIDEOS=(waypoints gps_outage wind emergency obstacle_field)

mkdir -p "$DATA_DEST"
for name in "${SCENARIOS[@]}"; do
  cp "$JSON_DIR/$name.json" "$DATA_DEST/$name.json"
  echo "copied $name.json"
done

cp "$JSON_DIR/validation.json" "$DATA_DEST/validation.json"
echo "copied validation.json"

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
