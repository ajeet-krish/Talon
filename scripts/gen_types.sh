#!/usr/bin/env bash
# Regenerate the TypeScript type mirrors from the Rust core (Phase 5c, D23).
#
# The schema and config types in core/src/ carry ts-rs derives behind the
# `ts-export` feature. This script runs the ignored generator test once
# (`core/tests/ts_export.rs`) to write src/types/generated/talon_core.ts, then
# reports the path. Run it from anywhere; it always resolves the repo root from
# its own location.
#
# After changing a schema type in core/src/, re-run this and commit the result.
# CI regenerates and fails on a diff, so the committed mirror can never drift
# from the Rust source of truth.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(dirname "$SCRIPT_DIR")"
OUT_DIR="$ROOT_DIR/src/types/generated"
OUT_FILE="$OUT_DIR/talon_core.ts"

cd "$ROOT_DIR"

# Fail loudly rather than emitting a confusing cargo error if the pieces this
# script depends on are missing.
if [ ! -d "$ROOT_DIR/core" ]; then
    echo "error: core/ not found at $ROOT_DIR/core; run from the Talon repository" >&2
    exit 1
fi

if ! grep -q '^ts-export' "$ROOT_DIR/core/Cargo.toml"; then
    echo "error: the ts-export feature is missing from core/Cargo.toml" >&2
    echo "       expected: ts-export = [\"dep:ts-rs\"]" >&2
    exit 1
fi

# Run the single ignored generator. --ignored executes only #[ignore] tests;
# --features ts-export turns on the ts-rs derives that feed decl().
cargo test -p talon-core --features ts-export \
    --test ts_export -- --ignored --nocapture

if [ ! -f "$OUT_FILE" ]; then
    echo "error: expected $OUT_FILE to be written, but it is missing" >&2
    echo "       check core/tests/ts_export.rs" >&2
    exit 1
fi

echo ""
echo "wrote $OUT_FILE"
