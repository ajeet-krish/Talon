#!/bin/bash
# Build the Talon FlightLab WebAssembly kernel: the Rust GNC core
# (talon-core) wrapped by wasm-bindgen, emitted as an ES module plus a .wasm
# into public/wasm-rust/ where the app loads it at runtime.
#
# Needs wasm-pack and the wasm32-unknown-unknown target. Both are installed on
# demand below where possible.
set -e

cd "$(dirname "$0")/.."

if ! command -v wasm-pack >/dev/null 2>&1; then
    echo "Error: wasm-pack not found. Install with: brew install wasm-pack" >&2
    exit 1
fi

if command -v rustup >/dev/null 2>&1; then
    if ! rustup target list --installed | grep -q '^wasm32-unknown-unknown$'; then
        echo "Adding the wasm32-unknown-unknown target..."
        rustup target add wasm32-unknown-unknown
    fi
fi

echo "Building the Rust WASM kernel..."

# The out dir is resolved relative to the crate, so this lands in
# public/wasm-rust/ at the repository root.
wasm-pack build wasm \
    --release \
    --target web \
    --out-dir ../public/wasm-rust \
    --out-name talon_core

# wasm-pack writes npm publishing scaffolding alongside the module. The app only
# ships the module and the binary, and a ".gitignore" containing "*" inside a
# committed asset directory would hide exactly the files we do want to commit.
rm -f public/wasm-rust/package.json public/wasm-rust/.gitignore

echo ""
echo "WASM build complete:"
ls -lh public/wasm-rust
