# Talon Reviews

Last updated: 2026-10-18

## PyO3 bindings and the C++ retirement (FlightLab Phase 5b, 2026-10-18)

Scope: `bindings/` (`Cargo.toml`, `build.rs`, `build.sh`, `src/lib.rs`,
`src/convert.rs`, `src/types.rs`, `src/control.rs`, `src/estimation.rs`), the
two call sites changed in `python/talon/mavlink_sil.py`, the deletions
(`core/include/talon/`, `core/tests/`, the root `CMakeLists.txt`,
`bindings/pytalon.cpp`, `wasm/CMakeLists.txt`, `wasm/src/exports.cpp`,
`public/wasm/`, `src/live/{guidance,mission,motors,sensors,stateMachine}.ts`
with their tests, `src/live/math.ts`), the trimmed `src/live/constants.ts`, and
the rewritten `.github/workflows/ci.yml`.

Verdict: PASS.

### Correctness

- PASS: the 186 pytest tests, written against the C++ kernel and unedited
  through the port, pass against the Rust `_core`. Numeric parity rests on that
  pre-existing suite rather than on a fresh oracle.
- PASS: the two covariance call sites sum the same three diagonal entries in
  the same order (`P00 + P11 + P22`), so the convergence flag and the recorded
  position variance are unchanged, and the 200 Hz path no longer builds a
  15x15 nested list per step.
- PASS: the fault-injection rewrite preserves the one-shot +25 m north
  corruption exactly: copy the position, offset index 0, assign the field,
  write the nominal back into the filter. The innovation gate then rejects
  every later fix, which is what makes the corruption persist and
  `ekf_diverged` trip.
- PASS: `test_controller.py::test_reset_makes_output_repeatable` still passes,
  which is the assertion that each `step()` returns an owned command rather
  than a view into controller state.
- PASS: `src/live/math.ts` was only deletable after both importers (`sim.ts`,
  `types.ts`) moved `Vec3` to the schema type; vitest and the production build
  were re-run after the swap, and the wasm smoke test still passes.
- PASS: a clean `./wasm/build.sh` reproduces the committed
  `public/wasm-rust/` bundle byte for byte, so the drift check added to CI is
  meaningful rather than flaky.

### Conventions

- PASS: no em dashes, double hyphens used as dashes, or dash equivalents in
  the new files.
- PASS: the binding crate holds no autopilot logic; every method delegates
  into `talon-core`.
- PASS: `bindings/` names only the core's own type aliases (`Vec3`, `Mat3`,
  `Matrix15`, `Vector6`), so it never names `nalgebra` and cannot drift from
  the core's linear-algebra version.

### Maintainability

- PASS: the retired files are deleted rather than commented out, with git
  history as the record, and AGENTS.md notes the deletion so a later session
  does not go looking for `core/include`.
- PASS: AGENTS.md, README, architecture, the module map, and CI now describe
  the same three-crate workspace; no document still tells a reader to run
  cmake or to build the Emscripten kernel.
- PASS: the extension build resolves its own interpreter
  (`.cargo/config.toml` plus `bindings/build.sh`), so `cargo test` and
  `uv run pytest` cannot silently disagree about which Python they target.

### Security

- PASS: no hand-written `unsafe` in the crate; PyO3 generates the FFI shims.
- PASS: setter input is validated. `vec3_from` reports a `PyValueError` naming
  the argument when fewer than three numbers arrive, and the conversion reads
  into a fixed three-element array with a counter, so a short or wrong-typed
  sequence can neither index out of bounds nor shift the vector.
- PASS: no new network, filesystem, or process surface. The extension reads
  numbers and returns numbers and new objects.
- PASS: the shared object lands at `python/talon/_core<EXT_SUFFIX>` and `*.so`
  is gitignored, so no binary is committed to the repository.

### Residual risk

- The CI drift check rebuilds the wasm bundle and diffs the generated JS and
  declarations. It depends on the pinned wasm-pack (v0.15.0) and the committed
  `Cargo.lock`. It was validated locally (a clean rebuild produces no diff) but
  not executed on GitHub from this session; the `.wasm` binary is deliberately
  excluded from the comparison.
- `cargo test --workspace` must link libpython for the binding tests on Linux.
  PyO3 adds the rpath and this works on macOS, but the Linux path was not
  exercised here.
- The pytest suite now needs the extension. CI sets `TALON_REQUIRE_CORE=1` so a
  missing `_core` fails the run instead of skipping it and reporting zero
  tests.

## Live ground station and pacing (FlightLab Phase 6, 2026-10-18)

Scope: `src/live/stream.ts` (sample codec and recorder), `src/live/pacing.ts`,
`src/live/readout.ts`, `src/live/view.ts`, the streaming passthroughs on
`src/live/sim.ts` and the declared wasm surface in `src/live/engine.ts`,
`src/components/live/` (scene, trail, airframe, field, rail, strip chart,
captions), the `RunView` rewrite, `scripts/smoke_live_wasm.mjs`, and the NED to
scene mapping in `src/utils/interpolation.ts` with its two dependants.

Verdict: PASS.

### Correctness

- PASS: the recorder's capacity is sized from the config
  (`ceil(duration / DT_CONTROL) + 64`). The smoke test drains all four configs and
  reports `drained == sample_count` with headroom unused, so a streamed delta can
  not be silently truncated. `isFull` exists as the honest failure mode if a
  config ever disagreed with the core.
- PASS: the streamed layout is a hand-mirrored contract with the Rust side, so
  the run view asserts `sample_stride()` from the loaded build against the local
  `SAMPLE_STRIDE` and disables itself with a message on a mismatch. This was a
  real hazard: the committed wasm build in the tree predated the streaming API,
  and without the check the page would have read `undefined` offsets and drawn a
  plausible but wrong flight. The build was regenerated in this phase.
- PASS: pacing does not drift. The step count is derived from
  `round((target - session_time) / DT)` each frame rather than accumulated, and
  the smoke test paces the hover config at 1x over 60 Hz frames: 3600 frames,
  60.00 s of wall time for 60.00 s of sim, maximum lag under one frame.
- PASS: a hidden tab can not teleport the vehicle. `tick` clamps a frame delta to
  `MAX_FRAME_MS` and the test drives a 30 s delta to prove it.
- PASS: changing the rate re-bases on the current sim time, so the vehicle never
  snaps forward or backward when the operator moves between 16x and 1x.
- PASS: `indexAt` floors rather than rounds. The original implementation seeded
  with `Math.round` and then corrected downward, which made its own doc comment
  ("nearest") false; it now estimates with `Math.floor` and walks both ways, and
  a live display can not sample a value the simulation has not produced.
- PASS: the trail wraps the recorder's array as its position attribute, and the
  draw range is what reveals it, so no geometry is rebuilt and no copy happens
  per frame. The colour attribute is written only for the newly recorded tail,
  tracked by `coloured.current`.
- FOUND AND FIXED IN THIS PHASE: `nedToScene` read the wrong NED component.
  It negated index 1, which is east, and passed index 2, which is down, through
  as depth, while its own comment said `[n, -d, e]`. Altitude therefore never
  appeared where it was drawn, which is the user-reported symptom (a vehicle that
  sits on the ground plane while the charts report it at 10 m). `deriveTelemetry`,
  `python/talon/plotting.py`, and the recorded climb profiles all agree that
  index 2 is down, so the scene was the outlier. Two dependants carried the same
  misreading: the drone's ground-track yaw was `atan2(north, down)`, and the wind
  field advected along east and down instead of east and up. All three are fixed,
  the mapping is now documented as a rotation with determinant +1, and the test
  that previously enshrined the bug asserts the basis mapping instead.
- PASS: unpausing, resetting, and re-running are all covered. Reset reuses the
  same recorder arrays in place (the renderer holds references to them) and
  clears them, which is why `LiveBuffer.reset` exists rather than a new instance.

### Conventions

- PASS: no em dashes, double hyphens used as dashes, or dash equivalents were
  introduced in code, comments, docs, or commit messages.
- PASS: TypeScript is `strict` with `noUncheckedIndexedAccess`; every indexed
  read in the new code goes through `?? fallback`. `tsc --noEmit` and the
  `tsconfig.build.json` project check are both clean.
- PASS: pure logic lives outside components (`stream.ts`, `pacing.ts`,
  `readout.ts`, `view.ts`) and is unit-tested in the node environment; no
  autopilot logic was added to the page, and the core still owns the flight.
- PASS: the design language matches the existing tool: mono uppercase labels at
  9 to 11 px with wide tracking, `border-grid` and `bg-panel` surfaces, the ink
  scale, signal cyan for active state, `tabular-nums` on every readout.
- PASS: phase colours come from the shared `PHASE_COLORS` palette and clearance
  tint from the shared `clearanceBand` and `CLEARANCE_COLORS`, so the live scene
  and the replay can not disagree about what a colour means.

### Maintainability

- PASS: the rail can not reuse `charts/LineChart.tsx` (its cursor and live value
  read the store clock), so `LiveSpark` is a deliberate, documented second strip
  chart rather than a silent divergence.
- PASS: `readout.ts` produces the rail's numbers from one function driven by a
  fake session in tests, instead of from JSX that a test can not reach.
- NOTE: the web app now assembles a scene twice, recorded and live. They share
  `GroundPlane`, the palette, and the clearance helpers, but a change to the
  scene language has to be made in both. Recorded in D26 as the accepted cost of
  not growing a `FlightData` document in the store mid-run.
- NOTE: `RunState` is declared in `TelemetryRail.tsx` and imported by the view.
  It is view state rather than recorder state, so it sits there, but it is the
  kind of type that would be happier in a small shared module if a third
  consumer appears.

### Performance

- PASS: the frame loop allocates nothing steady-state. `readReadout` reuses a
  module scratch frame, `poseAt` writes into a caller-owned frame, and the
  recorder fills preallocated arrays.
- PASS: the scene is `memo`ised on the scenario and the recorder, so the rail's
  10 Hz text updates do not reconcile the R3F tree, and the scene animates from
  the recorder's `clock` field rather than from React state.
- PASS: the step budget is capped (`MAX_STEPS_PER_FRAME`, 4000 plant steps or
  20 s of flight), so the maximum rate can not lock the tab inside one frame.
- PASS: the production build is clean; the single chunk-size warning is
  pre-existing (R3F plus three).

### Security

- PASS: no new attack surface. The only inputs are a same-origin committed
  `configs.json` and the wasm module from the Vite base URL; both are static
  assets of the deployment.
- PASS: no dynamic code evaluation, no `dangerouslySetInnerHTML`, and no string
  built from data reaches the DOM as markup. Scene labels are drawn to a canvas
  from config-supplied ids, which are committed values.
- PASS: the wasm session is released on completion, on reset, and on unmount, so
  a page session does not accumulate native objects.
- PASS: no untyped wasm call results are trusted structurally: every indexed
  read from a streamed block is range-checked by `?? fallback` and the block
  length is what bounds the loop.

### Residual risk

- No browser automation is available, so the visual result (a vehicle that
  visibly climbs, a trail that colours by phase, a readable rail) is verified by
  the node smoke test, the unit tests, and a `curl` pass over the served build,
  not by a rendered screenshot. A visual check in a browser is still worth doing
  before the phase is presented.
- The committed wasm build must be regenerated (`./wasm/build.sh`) whenever the
  core's streamed surface changes; the runtime stride check turns a forgotten
  rebuild into a visible message rather than wrong numbers, but it can not catch
  a reordering that keeps the stride.

## FlightLab Phase 3: unified entry point (2026-10-17)

Scope: the four-section shell in `src/views/` and `App.tsx`, the
new pure utilities (`src/utils/chart.ts`, `src/utils/telemetry.ts`,
`src/utils/format.ts`), the SVG chart (`src/components/charts/LineChart.tsx`),
the `validation.json` sidecar in `python/sims/run_all.py`, and the video staging
in `scripts/copy_data.sh`.

Verdict: PASS.

### Correctness

- PASS: `npm run build` (tsc project check plus vite production build) exits
  0; `dist/` contains `data/` (four scenarios plus `validation.json`) and
  `videos/` (five recordings).
- PASS: hooks order is unconditional in every view. `TelemetryView` and
  `MetricsView` call their `useMemo`/`useEffect` before any early return, so
  switching sections cannot violate the rules of hooks.
- PASS: `src/utils/chart.ts` decimation preserves local extremes
  (`chart.test.ts` proves a lone spike survives a four-point budget) and emits
  non-decreasing times, so the polyline cannot zigzag.
- PASS: `src/utils/telemetry.ts` normalizes signed zero on the altitude
  channel (`altitudeOf`), so a zero down value renders as `0` and not `-0.0`.
- PASS: `python/sims/run_all.py` writes `validation.json` from the same `ReportRow`
  list as the markdown report, so the two cannot disagree; the driver run
  reports 25/25 with the JSON sidecar present.

### Conventions

- PASS: no em dashes, double hyphens, or dash equivalents in any new source,
  test, doc, or script.
- PASS: C++/Python/TS conventions untouched; new TS uses `noUncheckedIndexedAccess`
  guards (`times[i] as number`, `?? [0, 0, 0]`) rather than non-null assertions.
- PASS: chart colors (`CHART_COLORS`) and verdict colors stay outside the
  reserved Wong flight-phase palette in `src/utils/colors.ts`, so phase color
  keeps a single meaning.

### Maintainability and performance

- PASS: one clearance module and one telemetry module feed scene, HUD, legend,
  charts, and metrics; no duplicated derivation.
- PASS: chart geometry is memoized on the payload; only `ChartCursor` and
  `LiveValue` subscribe to `currentTime`, so a playing scenario does not
  rebuild a roughly seven-hundred-point path per chart per frame (see
  decisions D13, D14).
- PASS: shared scenario metadata is centralized in `src/data/scenarios.ts` and
  consumed by the rail and the telemetry header (removes the duplicate record
  that previously lived in `ScenarioSelector.tsx`).
- PASS: `copy_data.sh` names both the scenario and recording lists explicitly,
  so a new scenario or recording is a one-line addition.

### Security review

- PASS: no new network surface. `loadValidation` fetches a static same-origin
  path built from `import.meta.env.BASE_URL`; scenario ids come from the fixed
  `SCENARIO_IDS` union and recording files from a fixed list, so no
  user-controlled string reaches a fetch or a `src`.
- PASS: no `dangerouslySetInnerHTML`, `eval`, `new Function`, or dynamic HTML.
- PASS: validation rows are rendered as React text nodes; the driver-supplied
  strings are escaped by React, not interpolated into markup.
- PASS: `LineChart` renders `NaN`-safe readouts via `formatMetric` (guards
  non-finite input), so a malformed sample cannot crash a chart.
