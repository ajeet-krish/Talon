#!/usr/bin/env bash
#
# Build the Python extension module (`talon._core`) from the Rust core.
#
# The Python analysis harness imports `talon._core`; the browser gets the same
# `talon-core` through `wasm/build.sh`. Both wrappers are thin: the autopilot
# lives in `core/` and nowhere else.
#
# Usage:
#   ./bindings/build.sh            # release, the default
#   ./bindings/build.sh debug      # unoptimised, for fast iterations
#
# The artifact is copied to `python/talon/_core<EXT_SUFFIX>`, next to the
# package that imports it, so `uv run pytest python/tests/` works without an
# install step. maturin remains the tool of choice for building a distributable
# wheel (`uv run maturin build --manifest-path bindings/Cargo.toml`).

set -euo pipefail

cd "$(dirname "$0")/.."

PROFILE="${1:-release}"

# Build against the interpreter the tests use: the extension suffix and the
# PyO3 ABI must match the venv.
if [ -x .venv/bin/python ]; then
    PYTHON_BIN="$PWD/.venv/bin/python"
elif command -v python3 >/dev/null 2>&1; then
    PYTHON_BIN="$(command -v python3)"
else
    echo "error: no python interpreter found (expected .venv/bin/python or python3)" >&2
    exit 1
fi

EXT_SUFFIX="$("$PYTHON_BIN" -c 'import sysconfig; print(sysconfig.get_config_var("EXT_SUFFIX") or ".so")')"

if [ "$PROFILE" = "release" ]; then
    PYO3_PYTHON="$PYTHON_BIN" cargo build -p talon-py --release --features extension-module
    TARGET_DIR=target/release
else
    PYO3_PYTHON="$PYTHON_BIN" cargo build -p talon-py --features extension-module
    TARGET_DIR=target/debug
fi

LIB="$TARGET_DIR/libtalon_py.dylib"
[ -f "$LIB" ] || LIB="$TARGET_DIR/libtalon_py.so"
[ -f "$LIB" ] || LIB="$TARGET_DIR/talon_py.dll"

cp "$LIB" "python/talon/_core${EXT_SUFFIX}"
echo "installed python/talon/_core${EXT_SUFFIX} from $LIB"
