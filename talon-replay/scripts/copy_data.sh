#!/usr/bin/env bash
# Refresh replay telemetry from the Talon simulator.
# From the Talon repo root: uv run python sims/run_all.py
# (regenerates plots/json/*.json), then from this app directory:
#   scripts/copy_data.sh
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
APP_DIR="$(dirname "$SCRIPT_DIR")"
SRC_DIR="$APP_DIR/../plots/json"
DEST_DIR="$APP_DIR/public/data"

SCENARIOS=(gps_outage wind emergency)

mkdir -p "$DEST_DIR"
for name in "${SCENARIOS[@]}"; do
  cp "$SRC_DIR/$name.json" "$DEST_DIR/$name.json"
  echo "copied $name.json"
done

echo "Telemetry refreshed into public/data/"
