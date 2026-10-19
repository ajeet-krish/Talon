# Talon FlightLab

Drone autopilot simulator: a Rust GNC core (RK4 dynamics, Dryden wind, cascaded PID, 15-state MEKF, NLGL guidance, reactive obstacle avoidance, waypoint mission, phase state machine, sensors, motors, and the SIL runner) that runs natively and in the browser through wasm-bindgen, with Python simulation and analysis tooling, and a React 19 + react-three-fiber web tool that replays, inspects, and flies live runs. The autopilot lives in `core/` and nowhere else: the browser (`wasm/`) and Python (`bindings/`, a PyO3 extension) are thin wrappers over it. The web app owns the repository root; the Python package, simulations, and tests live in `python/`.

## Quick Commands

```bash
# Rust core (the autopilot), from the repository root
cargo test -p talon-core                           # native GNC tests
cargo test -p talon-wasm                           # wasm-binding tests, run natively
cargo test -p talon-py                             # binding-level marshalling tests
./wasm/build.sh                                    # -> public/wasm-rust/talon_core.js + _bg.wasm

# Python harness
uv sync                                            # Python environment (.venv)
./bindings/build.sh                                # -> python/talon/_core<EXT_SUFFIX>
uv run pytest python/tests/ -q                     # pytest suites
uv run python python/sims/run_all.py               # validation driver (recorded data is frozen)

# Web app (from the repository root)
npm install
npm run dev                                        # vite dev server, base /Talon/ (localhost:5173/Talon/)
npm run build                                      # typecheck + production build into dist/
npm test                                           # vitest unit tests
npm run smoke:wasm                                 # headless live-run check against public/wasm-rust
scripts/copy_data.sh                               # refresh public/data + public/videos from the driver output

uv run python python/sims/export_live_configs.py   # -> public/data/live/configs.json
```

## Conventions

- No em dashes, double hyphens used as dashes, or dash equivalents anywhere: code, comments, docs, commit messages.
- Rust: standard Cargo layout and rustfmt defaults; snake_case functions, PascalCase types, SCREAMING_SNAKE constants; `Result` over panics; no autopilot logic outside `core/`.
- Python: snake_case, type hints, uv-managed dependencies; package lives under `python/talon/`.
- Frames: NED world, aerospace body, Hamilton scalar-first quaternions, thrust along -body-z. Single source of truth is `docs/conventions.md`.
- TypeScript (web app): `strict` with `noUncheckedIndexedAccess`; typecheck before build; pure logic in `src/utils/` with vitest coverage.

## Project Map

- `src/`: web app source. `views/` (Replay, Telemetry, Metrics, Videos, Run), `components/` (scene, live, charts, ui), `live/` (engine loader, sample recorder, pacer, display geometry, thin LiveSim seam), `utils/` (pure), `store/` (Zustand), `data/` (scenario + recording + live-run metadata), `hooks/`.
- `core/src/`: the Rust autopilot, `talon-core`. `sim.rs` (SIL runner), `scenario.rs` (browser config), `export.rs` (schema 1.1 document), `dynamics.rs`, `wind.rs`, `controller.rs`, `ekf.rs`, `guidance.rs`, `avoidance.rs` (keep-out zones, reactive law), `mission.rs`, `state_machine.rs`, `sensors.rs`, `motors.rs`, `battery.rs`, `rng.rs`, `math.rs`, `types.rs`, `constants.rs`.
- `wasm/`: wasm-bindgen crate exposing the stepping `Session` to the browser (`src/lib.rs`), built by `build.sh` into `public/wasm-rust/`.
- `bindings/`: PyO3 crate `talon-py` publishing the `talon._core` extension module over `talon-core`, built by `build.sh` into `python/talon/`.
- `python/talon/`: Python package. `guidance.py` (NLGL), `mission.py`, `state_machine.py`, `sensors.py`, `motors.py`, `battery.py`, `avoidance.py`, `mavlink_sil.py`, `plotting.py`, `export_json.py`, `animation.py`.
- `python/sims/`: seven `scenario_*.py` runs, `monte_carlo.py`, and `run_all.py` validation driver.
- `python/tests/`: pytest suites, including the Python versus `_core` parity harness (`test_parity_*.py`).
- `public/data/`, `public/videos/`: committed app assets, staged by `scripts/copy_data.sh`.
- `public/wasm-rust/`: committed wasm-bindgen output the live run loads.
- `docs/`: `conventions.md` plus `architecture.md`, `decisions.md`, `research.md`, `reviews.md`, `tests.md`, `videos/`, `portfolio/`.

## Status

| Phase | Description | Status |
|-------|-------------|--------|
| Phase 1 | Obstacle scenario + schema 1.1 (offline detour avoidance) | Complete |
| Phase 2 | Interactive obstacle scene (R3F + bloom + labels) | Complete |
| Phase 3 | Unified entry point (videos + scene + metrics) | Complete |
| Phase 4 | In-browser WASM run | Complete |
| Phase 5a | Rust `talon-core` library + native tests | Complete |
| Phase 5b | WASM + PyO3 bindings, retire the C++ and TypeScript autopilots | Complete |
| Phase 5c | Generated TypeScript type mirrors (ts-rs) | Complete |
| Phase 6 | GCS shell with live telemetry and teaching captions | Complete |
| Phase 7 | Altitude legibility in the scene | Complete |
| Phase 8 | Mission authoring (waypoints, wind, faults) | Not started |
| Phase 9a | Reactive avoidance in Rust: geometry, the law, loop wiring, tests | Complete |
| Phase 9b | One implementation (Python delegates to `_core`) + a browser demo config | Complete |
| Hardening | Wrap contract, lint and CI gates, Python versus Rust parity harness | Complete |

## Deep Docs

- [Architecture](docs/architecture.md)
- [Decisions](docs/decisions.md)
- [Research](docs/research.md)
- [Reviews](docs/reviews.md)
- [Tests](docs/tests.md)
