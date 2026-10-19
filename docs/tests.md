# Talon Test Results

Last updated: 2026-10-18

## Reactive avoidance, native (Phase 9a, 2026-10-18)

| Suite | Command | Result |
|-------|---------|--------|
| Rust core (cargo) | `cargo test -p talon-core` | 210 pass (was 181) |
| Wasm binding (cargo) | `cargo test -p talon-wasm` | 12 pass (was 11) |
| PyO3 binding (cargo) | `cargo test -p talon-py` | 2 pass |
| Python harness (pytest) | `uv run pytest python/tests/ -q` | 186 pass |
| Web unit tests (vitest) | `npm test` | 131 pass (10 files) |
| Typecheck + production build | `npm run build` | clean |
| Live wasm smoke test | `npm run smoke:wasm` | all checks pass, 4 of 4 configs |
| Wasm bundle rebuild | `./wasm/build.sh` | only `talon_core_bg.wasm` changed |

The 29 new tests are 23 in `core/src/avoidance.rs`, 5 closed-loop in
`core/src/sim.rs`, and 1 in `core/src/scenario.rs`. The Python suite and vitest
counts are unchanged: the binding surface and the TypeScript layers were not
touched, and the rebuilt wasm glue is byte-identical apart from the binary.

### New core tests: geometry and the law (`core/src/avoidance.rs`, 23)

- Geometry (11): the leg-to-zone distance for a centre on the leg, off it, and
  clamped to each endpoint, plus the degenerate leg; the intersection test
  clear, penetrating, and for the near miss a margin catches; the detour point
  clear of the zone by exactly `radius + margin + buffer` and at the leg's
  altitude; the on-leg case stepping perpendicular; `plan_avoidance_mission`
  inserting one detour on a threatened leg, none on a clear one, and copying a
  single waypoint; the minimum surface clearance over a trajectory, including
  the empty-input cases; the signed surface clearance inside and outside.
- Tuning (2): negative margin, buffer, and lookahead each rejected with their
  own error; the lookahead floored at 2 m and scaling with ground speed.
- The law (10): disabled never takes over; no zones means no law; a zone beyond
  the lookahead is left alone and taken when it closes; a zone abreast of the
  leg origin or past its end is not on the leg; a zone behind the vehicle is not
  re-engaged; the latch holds on approach and releases beside the zone and
  clear; the latch does not release inside the margin; a new waypoint retires
  the latch; leaving the WAYPOINT phase clears it; several zones on one leg are
  taken one after another, asserting exactly two activations.

### New closed-loop tests (`core/src/sim.rs`, 5)

- `zones_alone_do_not_change_the_flown_path`: truth histories with and without
  zones are identical, sample for sample, with the law off. The frozen artifacts
  are safe by construction.
- `the_same_mission_without_the_law_flies_through_the_zone`: the planned path
  breaches the zone (minimum clearance minus 1.04 m), so the next test's result
  is attributable to the law.
- `the_law_flies_around_the_zone_and_completes_the_mission`: one bypass taken,
  the mission completes and lands, and the minimum truth clearance is 3.24 m
  against the 2 m margin.
- `the_law_clears_a_zone_on_a_later_leg`: the same, for a zone sitting on the
  second leg rather than the first.
- `negative_avoidance_tuning_is_rejected_at_construction`: typed error from
  `SimRunner::new`.

### New scenario and wasm tests

- `core/src/scenario.rs`: the `avoidance` block absent yields the documented
  disabled defaults; a partial block (`{"enabled": true}`) keeps the rest of
  the defaults; and the projection carries the tuning and the zones through to
  the runner.
- `wasm/src/lib.rs`: a scenario JSON string with a straight-through mission and
  the law on runs through `Session` and reports one or more activations, a
  finite clearance of at least 2 m, and a completed mission. The existing
  summary test now also asserts the no-zones case reports `null` clearance and
  zero activations.

### Bundle and boundary checks

- The stride and the streamed sample layout are untouched, so the hand-mirrored
  TypeScript decoder needed no change and the run view's stride assertion still
  passes.
- `./wasm/build.sh` reproduced the committed bundle with only
  `talon_core_bg.wasm` differing, which is the expected consequence of a core
  change with an unchanged exported surface.

## Altitude legibility in the live scene (Phase 7, 2026-10-18)

| Suite | Command | Result |
|-------|---------|--------|
| Web unit tests (vitest) | `npm test` | 131 pass (10 files) |
| Typecheck + production build | `npm run build` | clean, 647 modules |
| Live wasm smoke test | `npm run smoke:wasm` | all checks pass, 4 of 4 configs |
| Served build | `npm run preview` + curl | 200 for the page, the wasm module, and the live configs |

Vitest moved from 122 to 131 (9 new tests). Nothing else in the suite changed:
the phase touches the live scene only, so the Rust and Python suites were not
re-run for it.

### New tests

Nine of the new tests are added to `src/live/view.test.ts`:

- `altitudeLadder` (6). A 10 m mission gets 2 m rungs, 2 through 10, with 10
  flagged as the flight level. A waypoint altitude is flagged too
  (`takeoff 10, waypoint 6`). An off-grid ceiling is appended rather than lost
  (`takeoff 7` gives rungs 2, 4, 6, then the ceiling at 7). An off-grid
  waypoint altitude is flagged over the grid rung below it (`takeoff 9,
  waypoint 6` gives 2, 4, 6, 8, 9 with 6 and 9 flagged). A large mission stays
  bounded (100 m gives at most seven rungs, the highest exactly 100). A mission
  that never leaves the ground gets no ladder and no altitudes.
- `niceStep` (2). The interval snaps down to something an operator would print
  (16.67 to 10, 2.4 to 2, 0.6 to 0.5) and falls back to the finest interval
  when the value is below all of them.
- Shipped configs (1). Every scenario in the committed
  `public/data/live/configs.json` produces a non-empty ladder whose highest
  rung is its highest flown altitude and is flagged. This is the drift check
  that ties the ruler to the scenarios the app actually loads.

### Not covered

The ladder's geometry, proportions, and colours, and the plumb line's width,
have no automated coverage: there is no browser in CI and `npm run smoke:wasm`
flies the wasm engine rather than the React Three Fiber tree. The decision
logic is pure and tested; the drawing is guarded by the typecheck in
`npm run build` and recorded as a residual risk in `docs/reviews.md`. A human
has to look at the page.

## Rust core behind both wrappers (FlightLab Phase 5b, 2026-10-18)

| Suite | Command | Result |
|-------|---------|--------|
| Rust core (cargo) | `cargo test -p talon-core` | 181 pass |
| Rust wasm bindings (cargo) | `cargo test -p talon-wasm` | 11 pass, run natively |
| PyO3 bindings (cargo) | `cargo test -p talon-py` | 2 pass |
| Whole workspace (cargo) | `cargo test` | 194 pass across the three crates |
| Python against the Rust `_core` (pytest) | `uv run pytest python/tests/ -q` | 186 pass |
| Web unit tests (vitest) | `npm test` | 122 pass (10 files) |
| Typecheck + production build | `npm run build` | clean, 646 modules |
| Live wasm smoke test | `npm run smoke:wasm` | all checks pass, 4 of 4 configs |
| wasm bundle determinism | `./wasm/build.sh`, then `git status public/wasm-rust` | clean (byte-identical rebuild) |

### The parity proof

The 186 pytest tests were written against the C++ kernel and were not touched
while the kernel moved to Rust. They now run against `talon._core`, the PyO3
extension over `talon-core`, and pass unchanged. That is the evidence that let
the C++ stack go, rather than a separate golden-vector harness.

Two call sites changed on the Python side, neither altering numerics:

- `mavlink_sil._update_derived` and `mavlink_sil._record_history` read the
  position variance through the new `Ekf.position_covariance_trace()` and
  `Ekf.position_variance()`, which sum the same three diagonal entries in the
  same order. The 200 Hz convergence check no longer converts a 15x15 nested
  list every step.
- `inject_ekf_divergence` uses `nominal()` plus `set_nominal()` instead of the
  `mutable_nominal()` aliasing hook, which PyO3 cannot express safely.

### New binding tests

- `bindings/src/convert.rs` (2): the 3-vector round trip preserves index order,
  and the 3x3 round trip is row major. These run without a GIL, so
  `cargo test -p talon-py` needs no interpreter.

### Deleted suites

Retired with the C++ stack: 34 Google Tests (`./build/talon_tests`). Retired
with the TypeScript autopilot: 13 vitest tests across `guidance.test.ts`,
`mission.test.ts` and `motors.test.ts`. The same laws are covered in Rust by 16
`guidance`, 13 `mission`, 15 `motors`, 13 `sensors` and 21 `state_machine`
tests. Vitest therefore fell 135 to 122 and the file count 13 to 10.

## Live ground station and pacing (FlightLab Phase 6, 2026-10-18)

| Suite | Command | Result |
|-------|---------|--------|
| Rust core (cargo) | `cargo test -p talon-core` | 181 pass |
| Rust wasm bindings (cargo) | `cargo test -p talon-wasm` | 11 pass, run natively |
| Python (pytest) | `uv run pytest python/tests/ -q` | 186 pass |
| Web unit tests (vitest) | `npm test` | 135 pass (13 files) |
| Typecheck + production build | `npm run build` | clean, 646 modules |
| Live wasm smoke test | `npm run smoke:wasm` | all checks pass, 4 of 4 configs |
| Served build | `npm run preview` + curl | 200 for the page, wasm module, `.wasm`, and configs |

### New frontend tests (Phase 6)

- `src/live/stream.test.ts` (18): the codec closes the stride with the last
  field and reads every named offset back; a delta is appended and truncated at
  capacity; `indexAt` floors to the newest sample at or before the requested time
  and clamps at both ends (`0.016` resolves to the sample at `0.01`, not the one
  at `0.02`, because a live display may not read ahead); `poseAt` interpolates
  position, velocity, and estimate between bracketing samples, holds at the
  newest sample rather than extrapolating, takes the shortest attitude path
  across a quaternion sign flip, and refuses the empty buffer; each sample is
  tagged with the phase in effect when it was recorded, including as the core's
  timeline grows in later frames; the derived series are one per documented name
  and carry the right values from a written block.
- `src/live/pacing.test.ts` (13): nothing advances before start; 1x is one
  second of sim per second of wall time; the rate scales the target (1, 4, 16);
  `Infinity` targets the end with no wall time; pause holds and resume continues;
  a rate change re-bases on the current sim time so the vehicle never snaps; the
  run never passes the end; a 30 s tab-switch delta is clamped to `MAX_FRAME_MS`
  instead of teleporting; a negative or zero delta is ignored; reset returns to
  zero stopped; a run can start from a non-zero sim time; the rate labels are
  `1x`, `16x`, `MAX`.
- `src/live/view.test.ts` (10): the mission frame includes the takeoff column,
  contains every waypoint and the far edge of each keep-out shell, and falls back
  to a readable volume for a degenerate config; `rollPitchYaw` reads level flight
  as zero, nose-up pitch as positive, a right roll as positive, a right turn as
  positive heading, and normalizes a non-unit quaternion; the scene-from-NED
  quaternion is the documented quarter turn.
- `src/live/sim.test.ts` (8 added): the streaming passthroughs. `sampleCount`,
  `drain` (with the from-index forwarded to the session), `phases` (including a
  malformed timeline degrading to an empty list), `gpsCounts`, and `ekfFlags`.
- `src/utils/interpolation.test.ts` (2 replaced by 3): `nedToScene` puts altitude
  on the up axis, keeps a level vehicle on the ground plane, and maps the NED
  basis onto the scene basis (north to +x, east to +z, down to -y), which is the
  assertion the old test was missing.

### Live wasm smoke test (`npm run smoke:wasm`)

Flown through the built `public/wasm-rust/` module the way the page drives it,
one frame of steps at a time, draining each delta:

| Config | Samples | Last time | Last altitude | Phases | Outcome |
|--------|---------|-----------|---------------|--------|---------|
| hover | 6000 | 59.99 s | 9.99 m | 4 | holds altitude |
| waypoints | 12000 | 119.99 s | 0.00 m | 8 | 5 of 5 waypoints, landed |
| obstacle_field | 7000 | 69.99 s | 0.00 m | 8 | 3 keep-out zones in the document, landed |
| wind | 6000 | 59.99 s | 9.99 m | 4 | holds altitude |

Every config: the streamed stride is 36 doubles and matches the page's decoder,
one sample is recorded per control step, draining returns exactly the recorded
samples with each delta aligned to the cursor, the recorder's capacity is never
exceeded, the stepped document is schema 1.1 with every sample present, the
estimator converges without diverging, and the last sample lands one control step
short of the requested duration. A paced run of the hover config at 1x over
60 Hz frames reaches the end in 3600 frames (60.00 s of wall time for 60.00 s of
sim) and never lags the wall clock by more than a frame, which is the check that
the per-frame step rounding does not drift.

## In-browser live run (FlightLab Phase 4, 2026-10-17)

| Suite | Command | Result |
|-------|---------|--------|
| C++ kernel (Google Test) | `./build/talon_tests` | 34 pass |
| Python (pytest) | `uv run pytest python/tests/ -q` | 186 pass |
| Web unit tests (vitest) | `npm test` | 80 pass |
| Web typecheck + build | `npm run build` | clean |
| Live run diagnostics | `LiveSim.runToCompletion()` for all four configs | 4 of 4 fly to completion |

### What the Phase 4 tests cover

Parity against the Python reference is pinned by
`fixtures/live_fixtures.json` (generated by
`python/sims/export_live_configs.py` from the Python modules themselves):

- `src/live/guidance.test.ts` (7): five NLGL leg cases match the Python
  `NLGLGuidance.compute_leg` outputs to 9 decimals (mid-leg, degenerate
  zero-length leg, behind-A recovery, below the Nu2 ground-speed floor, and
  a crab-wind case), plus the look-ahead floor and the wrapped-course range.
- `src/live/mission.test.ts` (2): a five-position approach sequence matches
  the Python `WaypointMission` active index, latch state, and remaining
  distance; an empty waypoint list is rejected.
- `src/live/motors.test.ts` (4): the closed-form inverse mixer matches
  Python's `motor_mixing` for three commands, the forward mixer matches,
  the inverse/forward pair round-trips to 9 decimals, and a five-step
  first-order lag sequence matches `MotorModel`.
- `src/live/kernel.test.ts` (4): the WASM kernel reproduces
  `talon.propagate` (one RK4 step to 9 decimals), the EKF predict/update
  (accepted flag plus position and velocity to 7 decimals), and the
  cascaded PD command (thrust and moment to 7 decimals), and reports
  `hover_thrust = mass * g`. Skipped automatically when
  `public/wasm/talon.js` is absent.
- `src/live/sim.test.ts` (3): the full live SIL for the hover config reaches
  10 m and converges the EKF (RMSE < 0.5 m); the obstacle-field config flies
  the circuit with all keep-out zones cleared (min 3-D surface clearance
  2.03 m, matching the 2.0 m planner margin); and the hover trajectory
  tracks the Python SIL reference within a stated tolerance (statistical
  agreement, since the noise streams differ by design; skipped when
  `fixtures/sil_reference.json` is absent).

Deterministic diagnostics from `LiveSim.runToCompletion()` (same 200 Hz /
100 Hz / 10 Hz rates as the Python SIL):

| Config | Samples | Phases | Waypoints | EKF RMSE | Min clearance |
|--------|---------|--------|-----------|----------|---------------|
| hover | 6000 | arm, takeoff, cruise | n/a | 0.094 m | n/a |
| waypoints | 12000 | arm, takeoff, cruise, waypoint, rtl, land, idle | 5 of 5 | 0.087 m | n/a |
| obstacle_field | 7000 | arm, takeoff, cruise, waypoint, rtl, land, idle | 8 of 8 | 0.091 m | 2.03 m |
| wind | 6000 | arm, takeoff, cruise | n/a | 0.094 m | n/a |

Known limits (documented, not defects): the browser noise realization is
statistically equivalent to, not bit-identical with, numpy's; fault
injection and battery drain are not exercised live; the `wind` config holds
station without a mission, so it ends in cruise like hover.

## Repository restructure (2026-10-17)

The web app moved to the repository root, the C++ core to `core/`, and the
Python package, simulations, and tests to `python/`. Every suite was re-run at
the new paths; the only code changes were path configuration.

| Suite | Command | Result |
|-------|---------|--------|
| C++ kernel (Google Test) | `./build/talon_tests` | 34 pass |
| Python (pytest) | `uv run pytest python/tests/ -q` | 186 pass |
| Config + store + utils (vitest) | `npm test` (repo root) | 60 pass |
| Validation driver | `uv run python python/sims/run_all.py --skip-video --skip-fingerprint` | 25/25 checks, 7/7 scenario asserts |
| Typecheck + production build | `npm run build` | clean |

Path-only changes: `CMakeLists.txt` (`core/include`, `core/tests`, and the
`python/talon` extension output dir), `pyproject.toml` (packages, testpaths,
pythonpath), `python/sims/run_all.py` (`PY_ROOT`/`REPO_ROOT`, pytest path),
`.github/workflows/ci.yml` (pytest path plus a new web job), and
`scripts/copy_data.sh` (repository-root paths).

## FlightLab Phase 3: unified entry point (2026-10-17)

| Suite | Command | Result |
|-------|---------|--------|
| C++ kernel (Google Test) | `./build/talon_tests` | 34 pass |
| Python (pytest) | `uv run pytest python/tests/ -q` | 186 pass |
| Config + store + utils (vitest) | `npm test` (repo root) | 60 pass |
| Validation driver | `uv run python python/sims/run_all.py --skip-fingerprint` | 25/25 checks, 7/7 scenario asserts |
| Typecheck | `npm run typecheck` | clean |
| Production build | `npm run build` | clean, 629 modules |

### New frontend tests (Phase 3)

- `src/utils/chart.test.ts` (10): decimation returns a copy when the series
  fits, reduces below the budget while keeping time order, and preserves a
  lone spike; `valueDomain` pads normal ranges, gives a flat series height,
  and falls back for an empty input; `xScale`/`yScale` hit the plot edges and
  invert the value axis; `buildPath` is empty with no samples and starts with
  a move plus a line otherwise.
- `src/utils/telemetry.test.ts` (6): `deriveTelemetry` derives altitude,
  speed, EKF error, and 3-sigma from the samples, flags a zero wind channel
  as flat, omits the clearance series without keep-out zones, and adds it
  (with correct surface distances) when zones are present; `summarizeRun`
  reports duration, rate, phase count, terminal phase, EKF RMSE/max, maxima,
  distance flown, and minimum clearance.
- `src/utils/format.test.ts` (4): precision tracks magnitude, zero is plain,
  non-finite input renders `--`, and negatives keep their sign.
- `src/store/useFlightStore.test.ts` (3 added): `setView` switches sections;
  `loadValidation` caches a successful report and records an error status on
  a failed fetch.

### Regeneration verification

- `python/sims/run_all.py` regenerated `plots/json/*.json` plus `plots/json/validation.json`
  and `docs/videos/*.mp4`. The four previously committed MP4s were reproduced
  byte-for-byte (`git diff --stat -- docs/videos/` is empty); only the new
  `obstacle_field.mp4` was added.
- `scripts/copy_data.sh` staged five JSON documents into `public/data/` and
  five recordings into `public/videos/`; the production build copies both
  trees into `dist/`.
