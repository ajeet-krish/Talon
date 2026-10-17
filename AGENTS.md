# Talon

Single-vehicle UAV autopilot: a header-only C++17 GNC kernel (dynamics, Dryden wind, cascaded PD, 15-state MEKF) exposed through pybind11, with Python mission, SIL, and analysis tooling plus a TypeScript 3D flight replay.

## Quick Commands

```bash
uv run cmake -B build -DCMAKE_BUILD_TYPE=Release   # configure kernel, GTest, pybind11
uv run cmake --build build -j                      # build talon_tests and _core*.so into src/talon/
./build/talon_tests                                # Google Tests (C++ kernel)
uv sync                                            # Python environment (.venv)
uv run pytest tests/ -q                            # pytest suites
uv run python sims/run_all.py                      # validation driver: scenarios, plots, report
```

talon-replay (from its own directory): `npm install`, `npm run dev`, `npm run build`, `npm test`.

## Conventions

- No em dashes, double hyphens used as dashes, or dash equivalents anywhere: code, comments, docs, commit messages.
- C++: header-only C++17 under `cpp/include/talon/`, 4-space indent, K&R braces; snake_case functions, PascalCase types, SCREAMING_SNAKE constants.
- Python: snake_case, type hints, uv-managed dependencies; package lives under `src/talon/`.
- Frames: NED world, aerospace body, Hamilton scalar-first quaternions, thrust along -body-z. Single source of truth is `docs/conventions.md`.
- TypeScript (talon-replay): `strict` with `noUncheckedIndexedAccess`; typecheck before build.

## Project Map

- `cpp/include/talon/`: header-only kernel. `dynamics.hpp` (RK4), `wind.hpp` (Dryden), `controller.hpp` (cascaded PD), `ekf.hpp` (MEKF), `types.hpp`, `constants.hpp`.
- `cpp/tests/`: Google Test suites.
- `bindings/pytalon.cpp`: pybind11 module `talon._core`.
- `src/talon/`: Python package. `guidance.py` (NLGL), `mission.py`, `state_machine.py`, `sensors.py`, `motors.py`, `battery.py`, `mavlink_sil.py`, `plotting.py`, `export_json.py`, `animation.py`.
- `sims/`: six `scenario_*.py` runs, `monte_carlo.py`, and `run_all.py` validation driver.
- `tests/`: pytest suites.
- `docs/`: `conventions.md` plus `architecture.md`, `decisions.md`, `research.md`, `reviews.md`, `tests.md`.
- `talon-replay/`: React 19, react-three-fiber 9, Vite 6 replay application.

## Status

| Phase | Description | Status |
|-------|-------------|--------|
| Phase 1 | Obstacle scenario + schema 1.1 (offline detour avoidance) | Complete |
| Phase 2 | Interactive obstacle scene (R3F + bloom + labels) | Pending |
| Phase 3 | Unified entry point (videos + scene + metrics) | Pending |
| Phase 4 | In-browser WASM run | Pending |

## Deep Docs

- [Architecture](docs/architecture.md)
- [Decisions](docs/decisions.md)
- [Research](docs/research.md)
- [Reviews](docs/reviews.md)
- [Tests](docs/tests.md)
