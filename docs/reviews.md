# Talon Reviews

Last updated: 2026-10-18

## One avoidance implementation (Phase 9b, 2026-10-18)

Scope: `bindings/src/avoidance.rs` (new), `bindings/src/lib.rs` (registration),
`python/talon/avoidance.py` (rewritten as a thin re-export),
`python/sims/export_live_configs.py` (the fifth config), the regenerated
`public/data/live/configs.json`, `scripts/smoke_live_wasm.mjs` (the
scenario-specific acceptance), `src/live/types.ts` (the optional block),
`src/live/view.test.ts` (the drift guard), and the docs.

Verdict: PASS. The removal of the second implementation is proved by the frozen
artifacts, not asserted.

### Correctness

- PASS: parity. `python/tests/test_avoidance.py` is untouched and all 20 of its
  cases pass against the delegated module, including the near-miss margin case,
  the degenerate leg, the on-leg perpendicular step, and the ValueError
  contract. The full Python suite is 186 pass, the same count as before.
- PASS: the exporter's output is the stronger check. Regenerating
  `public/data/live/configs.json` leaves `hover`, `waypoints`, `obstacle_field`,
  and `wind` identical field for field, so the recorded obstacle-field run's
  eight detoured waypoints are reproduced by the Rust planner through the
  Python wrapper. The diff is one changed `generated_utc` line plus the new
  config block.
- PASS: the new config actually exercises the law rather than carrying the
  block. Its straight legs cross all three zones, and the headless smoke run
  reports 3 activations, which is one per threatened leg and not a
  configuration that happens to sit clear.
- PASS: the demo's clearance is measured, not assumed. The smoke check reads
  the margin from the config and asserts the achieved minimum against it, so
  the acceptance cannot drift away from the artifact it is judging.
- PASS: the planner's single-detour-per-leg and single-waypoint behaviour is
  preserved through the boundary, and the wrapper rebuilds
  `talon.types.Waypoint` objects so callers that read `.position`, `.speed`,
  `.heading`, or `.radius` off the result are unaffected.
- PASS: the boundary reuses the core's own conversions. Coordinates go through
  `to_arr3`, so signed zero is canonicalized at the edge exactly as it is on
  every other boundary in the crate.

### Regression safety

- PASS: this phase changed no Rust, and the rebuilt wasm bundle is byte
  identical to the one committed in 9a. The four pre-existing live configs keep
  their exact JSON, so the browser's runs are unchanged.
- PASS: `fixtures/live_fixtures.json` regenerated identical, so the vitest
  parity suites that read it are unaffected.
- PASS: `cargo test --workspace` is 210 core, 12 wasm, 3 PyO3. The one new
  PyO3 test is a pure function of the margin, deliberately, because formatting
  a `PyErr` requires an initialized Python interpreter that the unit tests do
  not have; the message itself is asserted where it belongs, in the Python
  suite.
- PASS: the TypeScript layer is unchanged in behaviour. The new interface field
  is optional and additive, the drift guard reads the shipped file, and the
  typecheck confirms the hand-declared types still match the served JSON.
- PASS: the served build returns 200 for the page, the wasm module, and the
  live configs.

### Conventions

- PASS: no em dashes, double hyphens, or dash equivalents in any new source,
  test, or doc.
- PASS: the Python module keeps the docstring style of the other thin
  re-exports and states where the single source of truth lives, plus the one
  behavioural tightening (negative margins rejected up front).
- PASS: no autopilot logic in Python. `avoidance.py` contains no geometry, only
  marshalling into the forms the existing callers pass and expect back.

### Residual risk

- RESIDUAL RISK: the clearance *readout* remains two expressions of the same
  arithmetic, Rust in the summary and TypeScript in the per-frame rail, because
  the rail must update between frames. The avoidance *decision* is single
  source; the readout is documented as a deliberate mirror (D35).
- RESIDUAL RISK: the live demonstration is a visual claim. `npm run smoke:wasm`
  proves the run completes and holds clearance, but nothing automated can see
  that the scene draws the deviation, so a human still has to look at the page.

## Reactive avoidance, native (Phase 9a, 2026-10-18)

Scope: `core/src/avoidance.rs` (new: geometry, the reactive law, 23 unit
tests), `core/src/lib.rs` (module registration), `core/src/sim.rs` (`SimConfig`
gains `obstacles` and `avoidance`, the WAYPOINT arm substitutes the guidance
target, the result gains the two validation numbers, five closed-loop tests),
`core/src/scenario.rs` (`avoidance` block, projection, two tests),
`wasm/src/lib.rs` (summary fields, one JSON-scenario test), and the rebuilt
`public/wasm-rust/talon_core_bg.wasm`.

Verdict: PASS. The law is exercised closed loop, and the idle path is proved
bit-identical rather than argued.

### Correctness

- PASS: geometry parity. The ported functions are pinned by the same cases as
  `python/tests/test_avoidance.py`: centre on the leg, off it, clamped to both
  endpoints, a degenerate leg, clear against penetrating, and the near-miss
  that a 1 m margin catches (1.5 m lateral against a 1 m sphere). The
  detour point is asserted to sit `radius + margin + buffer` from the centre
  and to hold the leg's altitude, and the on-leg case is asserted to step
  perpendicular.
- PASS: `plan_avoidance_mission` inserts exactly one detour per threatened
  leg, none on a clear leg, and returns a copy for a single waypoint, matching
  the Python contract it replaces in 9b.
- PASS: the lookahead scales with ground speed and is floored (0 m/s gives
  2 m, 5 m/s gives 15 m at the default 3 s), so a hover still reacts.
- PASS: the law is stateful in one thing only, the latch, and the directive is
  otherwise a pure function of state, leg, and zones, so a run is reproducible.
  `the_same_seed_reproduces_the_same_flight` still passes.
- PASS: the latch cannot chatter. Latching requires the zone to be more than
  2 m ahead and releasing requires the vehicle level with the zone or at its
  bypass inside that band, so the two sets are disjoint in distance. A test
  that walks a two-zone leg asserts exactly two activations, none spurious.
- PASS: the law never releases inside the margin. The release is gated on the
  measured clearance, and the case of a position 2 m from the centre of a 4 m
  zone (clearance minus 2 m) is asserted to stay latched.
- PASS: a latch cannot outlive its leg. A new active waypoint index retires it,
  and so does a phase that is not WAYPOINT, both asserted.
- PASS: closed loop, a mission that cuts straight across a 4 m zone with the
  law on reaches every waypoint, lands, takes one bypass, and holds 3.24 m of
  clearance against the 2 m margin while the same mission with the law off
  breaches the zone (clearance minus 1.04 m). The control matters: it shows
  the clearance is the law's doing and not a property of the path.
- PASS: validity. A negative margin, buffer, or lookahead is rejected at
  construction with a typed error rather than silently clamped, and the
  scenario path rejects it before the run starts.

### Regression safety

- PASS: `zones_alone_do_not_change_the_flown_path` compares truth position
  histories sample for sample between a run with zones and a run without them,
  both with the law off, and asserts they are identical. The frozen recordings
  and the four shipped live configs cannot move because this phase exists.
- PASS: `cargo test --workspace` is 210 core, 12 wasm, 2 binding tests, up from
  181, 11, 2. The 29 new tests are the 23 in `avoidance.rs`, 5 closed-loop in
  `sim.rs`, and 1 in `scenario.rs`, plus the extended summary test.
- PASS: the Python suite is untouched at 186 pass, run against the rebuilt
  extension, since the binding surface did not change. The Python planner still
  owns the offline detour for now (9b collapses it).
- PASS: only `public/wasm-rust/talon_core_bg.wasm` changed when the bundle was
  rebuilt; `talon_core.js` and the `.d.ts` files are byte-identical, so the
  exported surface is unchanged and the page's hand-declared interface still
  matches (the typecheck confirms it).
- PASS: the field offsets in `SAMPLE_STRIDE` are untouched. The new numbers
  are reported on the summary and the result, not in the streamed sample
  block, so `src/live/stream.ts` and the stride assertion are unaffected.

### Conventions

- PASS: no em dashes, double hyphens, or dash equivalents in any new source,
  test, or doc.
- PASS: Rust conventions held: snake_case functions, PascalCase types,
  SCREAMING_SNAKE constants, doc comments on every public item, and every
  transcendental call routed through `math` (`hypot`) rather than the platform
  library, so native and wasm agree bit for bit.
- PASS: no autopilot logic outside `core/`: the loop only calls the law and
  substitutes the target; the wasm crate only reports what the core computed.

### Safety and physical limits

- PASS: the law works on the estimated state, the same state the rest of the
  loop flies on, and the reported clearance is measured on the truth
  trajectory, so a good-looking number cannot be bought with a wishful
  estimate.
- PASS: the bypass standoff includes a 0.5 m buffer beyond `radius + margin`,
  which covers the acceptance radius and ordinary tracking error, and the
  closed-loop run shows the achieved clearance landing above the margin rather
  than on it.
- RESIDUAL RISK: the release radius and the standoff buffer are heuristics
  (half the standoff, floored at 2 m). They are documented in the module and
  covered by two closed-loop scenarios, but a mission that pins a zone hard
  against a waypoint, or a zone wider than the detour reach, is not covered by
  a test and is not claimed to work. Multiple zones abreast of each other are
  handled sequentially, not by a single pre-planned detour.

## Altitude legibility in the live scene (Phase 7, 2026-10-18)

Scope: `src/live/view.ts` (new `AltitudeTick`, `niceStep`, `missionAltitudes`,
`altitudeLadder`), `src/live/view.test.ts` (the ladder suites, including one
that reads the committed `public/data/live/configs.json`),
`src/components/live/LiveAltitude.tsx` (new), the plumb-line frame logic and
header comment in `src/components/live/LiveAirframe.tsx`, and the mount in
`src/components/live/LiveScene.tsx`. The replay scene is untouched by design.

Verdict: PASS, with residual risk on the things no automated check can see.

### Correctness

- PASS: `altitudeLadder` always ends on the mission's ceiling. The grid is
  built at `floor(top / step)`, and the top (always a flown altitude) is
  appended when the grid falls short, so the highest rung equals
  `max(missionAltitudes(cfg))`. This is asserted directly, and again against
  every shipped config.
- PASS: the flight-level flag is not merely a rung existence check. A rung is
  flagged when it is within a tenth of the spacing of a flown altitude, so an
  off-grid waypoint altitude is flagged at its own ceiling rung rather than
  being lost, which the `takeoff 9, waypoint 6` case pins.
- PASS: altitude sign. Waypoint altitudes are read as `-w.position[2]`, the same
  NED-down to up conversion `nedToScene` uses. A reversed sign would have made
  every waypoint altitude negative, `missionAltitudes` would have filtered them
  out, and the ladder would have been empty; both the unit tests and the
  shipped-config test would have failed loudly.
- PASS: the rung count is bounded. Small missions get fine intervals and large
  ones coarse ones (`niceStep`), and the doubling loop caps the grid at
  `maxTicks` rungs, so a 100 m mission draws five rungs rather than twenty.
  Asserted for a 100 m takeoff altitude.
- PASS: the plumb line is scaled on all three axes, so the cylinder's radius
  scales with it; scaling only `y` (the previous behaviour) is what left the
  stem at a fixed world radius. The model radius is a single constant used in
  both the geometry argument and the scale arithmetic.
- PASS: visibility gates are unchanged in kind. The plumb line hides below
  0.4 m of height and the shadow fades out entirely, so a landed or idle
  vehicle shows no phantom cue.
- PASS: label textures are disposed on unmount and on scenario change
  (`textures` is the effect dependency), matching `LiveField`'s existing
  disposal discipline.

### Conventions

- PASS: no em dashes, double hyphens used as dashes, or dash equivalents in the
  new files.
- PASS: pure logic sits in `src/live/view.ts` with vitest coverage; the
  component holds geometry and materials only, which is the stated split for
  `src/live` and the reason the arithmetic is testable at all.
- PASS: the ladder reuses `labelTexture` rather than introducing a second
  labelling mechanism, so the "sprites, not DOM overlays" decision holds.
- PASS: no new hue. The measuring grid is steel, the axis and labels are the
  existing grid greys, and only the flight level uses the signal cyan that the
  route, the waypoint rings, and the nose already own, where "this is where we
  fly" is the same meaning.

### Maintainability

- PASS: one helper module owns the ladder's arithmetic, and the component is a
  direct drawing of its output, so a change to the rule is a change in one
  place with tests around it.
- PASS: the shipped-config test reads the same JSON the page fetches, so the
  ladder cannot drift from the scenarios the app actually serves.
- PASS: the header comments state why the ladder exists and what the three
  altitude cues each do, so the next reader does not add a fourth.

### Security

- PASS: no new input surface. The only text rendered is `String(altitude)` from
  numbers derived from committed config, and the sprites are canvas-drawn, so
  there is no injection path.
- PASS: no network, filesystem, or user data access in the scene; the test's
  `readFileSync` is a fixed repository path.

### Residual risk

- No browser runs in CI, and `npm run smoke:wasm` exercises the engine rather
  than the R3F tree, so nothing automated has looked at this geometry. Sizes,
  proportions, and colours were derived from the mission scale and the default
  frame distance (about 79 m for the circuit scenarios, roughly 11 px per metre)
  and the reasoning is recorded in D31, but a human still has to open the page.
- Sprite labels are world-scaled, so at the default frame for the two circuit
  scenarios the rungs read and the numbers need a pad zoom. This is inherent to
  the no-DOM-overlay decision, not a bug to fix here.
- The live and replay scenes now state altitude differently, because this phase
  was scoped to the live ground station. If the replay is brought up to match,
  the plumb-line logic in `LiveAirframe` and the ladder's helper are both
  reusable, but `DroneMarker` yaws to the ground track rather than showing
  attitude, so it is not a straight port.

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
