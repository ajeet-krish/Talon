#!/bin/bash
# Build the Talon FlightLab WASM kernel (the header-only C++ GNC kernel compiled
# with Emscripten + embind) into public/wasm/. Mirrors the zenith-web reference.
set -e

cd "$(dirname "$0")"

echo "Building Talon WASM kernel..."

# Find Python 3.10+ (required by Emscripten 6.x).
PYTHON=""
for candidate in python3.14 python3.13 python3.12 python3.11 python3.10; do
    if command -v "$candidate" &> /dev/null; then
        PYTHON="$(command -v "$candidate")"
        break
    fi
done
if [ -z "$PYTHON" ]; then
    echo "Error: Python 3.10+ required for Emscripten. Install via: brew install python@3.14"
    exit 1
fi

# Locate Emscripten: EMSCRIPTEN_DIR override, else newest Homebrew Cellar, else a
# known path.
if [ -z "${EMSCRIPTEN_DIR:-}" ]; then
    EMSCRIPTEN_DIR=""
    for dir in $(ls -1v /opt/homebrew/Cellar/emscripten/*/libexec 2>/dev/null); do
        if [ -f "$dir/emcc.py" ]; then
            EMSCRIPTEN_DIR="$dir"
        fi
    done
    if [ -z "$EMSCRIPTEN_DIR" ]; then
        EMSCRIPTEN_DIR="/opt/homebrew/Cellar/emscripten/6.0.12/libexec"
    fi
fi
if [ ! -f "$EMSCRIPTEN_DIR/emcc.py" ]; then
    echo "Error: Emscripten not found at $EMSCRIPTEN_DIR"
    echo "Set EMSCRIPTEN_DIR or install with: brew install emscripten"
    exit 1
fi

echo "Using Python: $PYTHON ($($PYTHON --version))"
echo "Emscripten:   $EMSCRIPTEN_DIR"

# Emscripten's emcc/em++ shell wrappers shell out to `python3`; make a
# directory where python3 resolves to our 3.10+ interpreter and put it first.
mkdir -p /tmp/emscripten-python-bin
ln -sf "$PYTHON" /tmp/emscripten-python-bin/python3
ln -sf "$PYTHON" /tmp/emscripten-python-bin/python
export PATH="/tmp/emscripten-python-bin:$PATH"
echo "Active python3: $(python3 --version) at $(which python3)"

LLVM_DIR="$EMSCRIPTEN_DIR/llvm"

rm -rf build
mkdir -p build

LLVM="$LLVM_DIR" \
$PYTHON "$EMSCRIPTEN_DIR/emcmake.py" cmake -B build -DCMAKE_BUILD_TYPE=Release

cmake --build build -j"$(sysctl -n hw.ncpu 2>/dev/null || nproc 2>/dev/null || echo 4)"

mkdir -p ../public/wasm
cp build/talon.wasm.js ../public/wasm/talon.js
cp build/talon.wasm.wasm ../public/wasm/talon.wasm.wasm

echo ""
echo "WASM build complete:"
echo "  public/wasm/talon.js"
echo "  public/wasm/talon.wasm.wasm"
