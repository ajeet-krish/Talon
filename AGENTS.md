# Talon FlightLab

Drone autopilot simulator: a header-only C++17 GNC kernel (dynamics, Dryden wind, cascaded PD, 15-state MEKF) exposed through pybind11, with Python mission, SIL, and analysis tooling, and a React 19 + react-three-fiber web tool that replays and inspects the results. The web app owns the repository root; the C++ core lives in `core/`; the Python package, simulations, and tests live in `python/`.

## Quick Commands

```bash
# Native core (from the repository root)
uv run cmake -B build -DCMAKE_BUILD_TYPE=Release   # configure kernel, GTest, pybind11
uv run cmake --build build -j                      # build talon_tests and _core*.so into python/talon/
./build/talon_tests                                # Google Tests (C++ kernel)
uv sync                                            # Python environment (.venv)
uv run pytest python/tests/ -q                     # pytest suites
uv run python python/sims/run_all.py               # validation driver: scenarios, plots, videos, JSON + validation report

# Web app (from the repository root)
npm install
npm run dev                                        # vite dev server, base /Talon/ (localhost:5173/Talon/)
npm run build                                      # typecheck + production build into dist/
npm test                                           # vitest unit tests
scripts/copy_data.sh                               # refresh public/data + public/videos from the driver output

# WebAssembly kernel for the in-browser live run (needs Emscripten)
./wasm/build.sh                                    # -> public/wasm/talon.js + talon.wasm.wasm
uv run python python/sims/export_live_configs.py   # -> public/data/live/configs.json + fixtures
```

## Conventions

- No em dashes, double hyphens used as dashes, or dash equivalents anywhere: code, comments, docs, commit messages.
- C++: header-only C++17 under `core/include/talon/`, 4-space indent, K&R braces; snake_case functions, PascalCase types, SCREAMING_SNAKE constants.
- Python: snake_case, type hints, uv-managed dependencies; package lives under `python/talon/`.
- Frames: NED world, aerospace body, Hamilton scalar-first quaternions, thrust along -body-z. Single source of truth is `docs/conventions.md`.
- TypeScript (web app): `strict` with `noUncheckedIndexedAccess`; typecheck before build; pure logic in `src/utils/` with vitest coverage.

## Project Map

- `src/`: web app source. `views/` (Replay, Telemetry, Metrics, Videos, Run), `components/` (scene, charts, ui), `live/` (TypeScript glue for the WASM run), `utils/` (pure), `store/` (Zustand), `data/` (scenario + recording + live-run metadata), `hooks/`.
- `wasm/`: Emscripten build of the kernel (`exports.cpp` embind `Kernel`, `CMakeLists.txt`, `build.sh` -> `public/wasm/`).
- `fixtures/`: Python-generated parity fixtures for the live-run tests (committed).
- `core/include/talon/`: header-only C++17 kernel. `dynamics.hpp` (RK4), `wind.hpp` (Dryden), `controller.hpp` (cascaded PD), `ekf.hpp` (MEKF), `types.hpp`, `constants.hpp`.
- `core/tests/`: Google Test suites.
- `bindings/pytalon.cpp`: pybind11 module `talon._core`.
- `python/talon/`: Python package. `guidance.py` (NLGL), `mission.py`, `state_machine.py`, `sensors.py`, `motors.py`, `battery.py`, `avoidance.py`, `mavlink_sil.py`, `plotting.py`, `export_json.py`, `animation.py`.
- `python/sims/`: seven `scenario_*.py` runs, `monte_carlo.py`, and `run_all.py` validation driver.
- `python/tests/`: pytest suites.
- `public/data/`, `public/videos/`: committed app assets, staged by `scripts/copy_data.sh`.
- `docs/`: `conventions.md` plus `architecture.md`, `decisions.md`, `research.md`, `reviews.md`, `tests.md`, `videos/`, `portfolio/`.

## Status

| Phase | Description | Status |
|-------|-------------|--------|
| Phase 1 | Obstacle scenario + schema 1.1 (offline detour avoidance) | Complete |
| Phase 2 | Interactive obstacle scene (R3F + bloom + labels) | Complete |
| Phase 3 | Unified entry point (videos + scene + metrics) | Complete |
| Phase 4 | In-browser WASM run | Complete |

## Deep Docs

- [Architecture](docs/architecture.md)
- [Decisions](docs/decisions.md)
- [Research](docs/research.md)
- [Reviews](docs/reviews.md)
- [Tests](docs/tests.md)
