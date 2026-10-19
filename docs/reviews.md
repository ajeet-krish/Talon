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

---

## Full codebase review (2026-10-18)

Whole-project review, not a phase diff: the Rust workspace (`core/`, `wasm/`,
`bindings/`), the Python harness (`python/`), the web app (`src/`), the scripts,
and the docs were all read against each other, and every suite was re-run.

### Verification (all commands from the repository root)

| Command | Result |
|---------|--------|
| `cargo test -p talon-core` | PASS, 210 tests, no warnings |
| `cargo test -p talon-wasm` | PASS, 12 tests, no warnings |
| `cargo test -p talon-py` | PASS, 3 tests, no warnings |
| `uv run pytest python/tests/ -q` | PASS, 186 tests |
| `npm test` | PASS, 132 tests (10 files) |
| `npm run build` | PASS, typecheck clean, one pre-existing chunk > 1200 kB warning |
| `cargo clippy --workspace --all-targets` | FAIL, exit 101 (details below) |
| `cargo fmt --all -- --check` | FAIL, exit 1 (details below) |

Clippy failures (rustc 1.99.0 toolchain):

- DENY-LEVEL ERROR, `core/src/guidance.rs:52`: `approx_constant`, the literal
  `-0.7071` approximates `f64::consts::FRAC_1_SQRT_2`. Aborts the lint of the
  workspace; `cargo clippy` therefore cannot pass at all today.
- talon-core warnings (18 in lib, 20 in lib test): `neg_cmp_op_on_partial_ord`
  at `core/src/battery.rs:86,89`, `core/src/motors.rs:127`, `core/src/scenario.rs:142`,
  `core/src/sim.rs:122`, `core/src/state_machine.rs:204`; `needless_update` at
  `core/src/dynamics.rs:131`; `needless_range_loop` at `core/src/motors.rs:154`;
  `type_complexity` at `core/src/sim.rs:353`; `field_reassign_with_default` at
  `core/src/sim.rs:426` and `core/src/dynamics.rs:197,227`; `collapsible_if` at
  `core/src/sim.rs:955,962,971,976,981,987,992,1007`.
- talon-py warnings (4): `type_complexity` at `bindings/src/avoidance.rs:144`,
  derivable `Default` impls at `bindings/src/estimation.rs:16,181`,
  `too_many_arguments` at `bindings/src/types.rs:268`. talon-wasm is clean.

Rustfmt failures: 87 hunks across 19 files, including production code
(`core/src/dynamics.rs` RK4 block, `core/src/ekf.rs:186,265,328`, `core/src/guidance.rs:154,178,196`,
`core/src/sim.rs` phase handlers, module order in `core/src/lib.rs`, import
order in `core/src/motors.rs`) plus a two-statements-on-one-line artifact at
`wasm/src/lib.rs:452` that looks like an editing slip. AGENTS.md claims rustfmt
defaults, so the tree does not currently meet its own stated convention.

### Check results

| Check | Verdict | Details |
|-------|---------|---------|
| Frames, quaternion, thrust conventions | PASS | dynamics.rs, ekf.rs, sensors.rs all match docs/conventions.md (NED, Hamilton scalar-first, body-to-NED, thrust along -body-z, specific force reads [0, 0, -g] at hover) |
| Integrator and dynamics | PASS | RK4 with per-step quaternion renormalization (dynamics.rs:88-137), quadratic drag on air-relative velocity (dynamics.rs:73-76), tested free fall, hover, and unit-norm persistence |
| EKF consistency | PASS | Joseph-form update with explicit symmetrization (ekf.rs:338-341), innovation gating that leaves state and P untouched on rejection (ekf.rs:314-318), right-multiplied error injection matching the controller convention |
| Controller saturation and anti-windup | PASS | back-calculation anti-windup scaling the integrator by T_max / T_unsat (controller.rs:150-155), degenerate a_des and y_b guards (controller.rs:159-177), closed-loop convergence test |
| Guidance and mission sequencing | PASS with one FAIL | NLGL law, clips, behind-A recovery, and altitude sign match the Python source and its defaults one for one (guidance.rs:74-89 vs guidance.py:126-135); mission acceptance and advance rules match mission.py and the 186-test suite. FAIL item: wrap_to_pi (below) |
| Avoidance geometry and reactive law | PASS | geometry matches the delegated Python tests case for case; the latch cannot chatter (D34 band), never releases inside the margin (avoidance.rs:439-450), retires on leg change (avoidance.rs:420-427); closed-loop proof in sim.rs |
| State machine transitions and guards | PASS | the transition table and every guard value match python/talon/state_machine.py exactly (soc > 0.30 arm, alt > takeoff - 0.5, soc < 0.20, outage > 5.0 s, home < 1.0 m, landed alt < 0.1 and |vz| < 0.1) |
| SIL loop thresholds | PASS | sim.rs handle_events matches mavlink_sil.py:826-864 value for value (takeoff, cruise hold, low battery, gps_lost, home, landed, emergency) |
| wasm boundary contract | PASS | src/live/engine.ts declares exactly the surface wasm/src/lib.rs exports; names and signatures agree |
| Sample stream contract | PASS | src/live/stream.ts SAMPLE_FIELD and SAMPLE_STRIDE = 36 mirror core/src/sim.rs sample_field exactly (0,1,4,7,11,14,17,20,23,26,29,32,33), and the runtime stride assertion covers the loaded build |
| PyO3 boundary contract | PASS | bindings/ marshals only, validates input (vec3_from, margin checks), exposes no second implementation; python defaults in guidance, mission, motors, sensors, battery, and constants all match the Rust values |
| TS type mirrors vs schema | PASS with one FAIL | src/types/flight.ts and src/live/types.ts field names match core/src/types.rs and scenario.rs; FAIL item: the e_ct comment (below) |
| Browser loop performance and resources | PASS | preallocated recorder buffers (stream.ts), pacing derived from clock difference (pacing.ts), MAX_STEPS_PER_FRAME = 4000 cap (RunView.tsx:28), rAF cancelled and the wasm session freed on unmount and completion (RunView.tsx:149,154-162,235), textures and geometries disposed in every scene component checked |
| Error handling, no panics on user input | PASS with note | scenario JSON and every tuning knob reject with typed errors; the only production `expect` sites are invariant-backed (ekf.rs:309, motors.rs:62, sim.rs:1048,1103,1143) |
| Dash policy (no em dashes or equivalents) | PASS | no em dash or dash-equivalent characters in rs, py, ts, tsx, md, sh, json, yml, html, css, toml; only `// ----` comment dividers, consistent with prior reviews' reading |
| Lint and format gates | FAIL | clippy deny-level error plus 22 warnings; rustfmt 87 hunks (verification table above) |
| Docs match code | FAIL | conventions.md still describes C++ headers and std::mt19937 (lines 4, 86); D4's "1.0 documents remain valid" claim contradicts export_json.py's validator; architecture.md and D18 describe vitest consuming fixtures/live_fixtures.json, which nothing reads since Phase 5b |
| Build and test suites | PASS | 543 tests green (210 + 12 + 3 + 186 + 132), production build and typecheck clean |

### Findings

| Severity | File:Line | Issue | Fix |
|----------|-----------|-------|-----|
| High | core/src/math.rs:48-50 | `wrap_to_pi` is wrong for theta < -pi: `fmod` keeps the sign of the numerator, so `wrap_to_pi(-4.0)` returns -4.0 while `guidance.py:110` (`wrap_angle`, pinned by test_guidance.py:20) returns +2.2832. The doc comment claims one-to-one parity with the Python helper; that claim is false. Flight behavior is unaffected today because the only consumer is `chi_d`, which is consumed through sin/cos (guidance.rs:209, controller.rs:167), but the diagnostics value is wrong and any future angle comparison is a silent bug. | Replace with `((theta + PI).rem_euclid(TAU)) - PI` or add the negative branch to the fmod form; port the four cases from python/tests/test_guidance.py:19-22 into math.rs tests (math.rs currently has no test module at all) |
| High | core/src/guidance.rs:52 | `cargo clippy` cannot pass: the literal `-0.7071` trips the deny-by-default `approx_constant` lint. 22 further style warnings in talon-core and talon-py (listed above). CI runs `cargo test` only, so nothing is red today, but the standard Rust quality gate is broken and would have to be fixed the day anyone gates on it. | Use `-std::f64::consts::FRAC_1_SQRT_2` (ArduPilot's own value) or an explicit `#[allow(clippy::approx_constant)]` with a comment that the truncated literal is deliberate; clear the warning list; add a clippy job to ci.yml |
| High | 19 files (see verification) | `cargo fmt --all -- --check` fails with 87 hunks including production code, while AGENTS.md claims rustfmt defaults. wasm/src/lib.rs:452 carries two statements on one line. | Run `cargo fmt --all`, review the diff, and gate `cargo fmt --check` in CI so it cannot regress |
| Medium | python/talon/{guidance,mission,state_machine,motors,sensors,battery}.py vs core/src/ | Six layers exist twice (documented in D28) with no parity test pinning the two implementations to each other. The numbers agree today (every default and guard value was compared in this review), but nothing fails when one side is tuned. `fixtures/live_fixtures.json` is regenerated by export_live_configs.py:63 and read by nothing since the TS autopilot tests were deleted in Phase 5b. | Either bind guidance/mission through PyO3 as happened for avoidance in D35, or repurpose the fixture pipeline as a pytest parity suite that runs the Python modules and the `_core` (or wasm session) over shared inputs and compares outputs. This completes the D28 "one implementation" story with evidence instead of convention |
| Medium | python/talon/export_json.py:62-83, 280-292, 324 | The documented backward-compatibility contract (D4: "old 1.0 documents remain valid; the validator checks the block only when present") is not what the code does: `obstacles` is in REQUIRED_KEYS and `validate_flight_json` raises on a missing key, then indexes `data["obstacles"]` unguarded. The module docstring (line 4) and the validator docstring (line 280) also still say "schema 1.0" while the code writes and requires 1.1. | Make `obstacles` optional in the validator (treat a missing key as empty, matching core/src/types.rs:285 `serde(default)`), or amend D4 and architecture.md to say 1.1 only; fix the two docstrings |
| Medium | docs/conventions.md:4, 86 | The "single source of truth" for conventions still describes "Every C++ header" and a `std::mt19937` wind generator. The stack is Rust/Python/TS and the generator is splitmix64 plus Box-Muller (core/src/rng.rs) with numpy RNG on the recorded path. | Rewrite those paragraphs for the current stack: libm-pinned transcendentals (D23), `crate::rng::Rng`, and the numpy caveat |
| Low | src/live/sim.ts:43-57, 213 | The page drops `min_obstacle_clearance_m`, `avoidance_activations`, and `gps_skipped` from the core summary (CoreSummary does not declare them) and recomputes clearance from the rounded document instead. This is the documented D35 residual, but the summary already carries the truth-measured value the smoke test asserts. | Read `min_obstacle_clearance_m` from the summary, keep the per-frame client readout as is (it must update between frames), and surface `avoidance_activations` on the run tiles so the demo proves its own behavior |
| Low | src/types/flight.ts:92 | `e_ct` is documented as "NLGL commanded-thrust error along the mission leg". It is the absolute cross-track error: sim.rs:1134 and mavlink_sil.py:966 both record `abs(diagnostics.e_ct)`. | Correct the comment to "| cross-track error |, sampled in WAYPOINT" |
| Low | core/src/types.rs:88-89 | `let _ = ww;` keeps an unused binding alive with a comment; the `ww` binding itself is dead code in `to_dcm`. | Drop `ww` from the destructuring |
| Low | core/src/ekf.rs:307-309 | `try_inverse().expect(...)` in the GPS update is invariant-backed today (R diagonal positive), but it is the one production panic reachable in principle if a future caller configures zero GPS noise and P goes numerically singular. | Return `false` (treat as a gated fix) instead of panicking; a flight core should be panic-free under every configuration |
| Low | core/src/wasm path, sim.rs:1247-1292 | `summary_json` and `document_json` both clone every history vector through `snapshot()` to compute a handful of scalars. Fine at completion (one call), wasteful if ever polled per frame. | Give SimRunner scalar accessors for the summary fields so a future mid-run poll does not clone the flight |
| Low | python/talon/battery.py:99, core/src/battery.rs:121, core/src/motors.rs:151 | API-edge drift between the two implementations: Python raises ValueError on a negative dt while Rust clamps to zero; Rust clamps an out-of-range initial soc while Python's behavior differs. Harness-only, no recorded run is affected. | Document the divergence in both modules or align the Python edges with the Rust "never panic the loop" posture |
| Low | README.md:5, docs/architecture.md:81 | The controller is described as "cascaded PD" but implements a position PID with an integrator (controller.rs:1, 138-141). | Say "cascaded PID" or "PD with position integral" wherever the loop is summarized |

No Critical findings.

### Overall evaluation

Strong. The single-source-of-truth discipline this project set for itself is
real and verifiable: the autopilot lives in one Rust library, both wrappers are
thin and marshalling-only, the schema types are the Rust types, the frozen
recorded artifacts prove the port rather than asserting it, and the avoidance
phase closed the last known duplication. The numerical core is careful work:
Joseph-form EKF with symmetrization, back-calculation anti-windup, exact
exponential motor lag, libm-pinned transcendentals for cross-target determinism,
and a reactive avoidance law with a latch that provably cannot chatter. Test
coverage is genuinely good (543 green tests, closed-loop as well as unit), the
error taxonomy is typed end to end, and the browser loop allocates nothing
steady state and disposes its GPU resources. For its stated purpose, a
simulator and teaching tool, the flight behavior is production quality.

Fragile. Three things carry latent risk rather than present bugs. The six
duplicated Python layers are correct today but unpinned; the wrap_to_pi defect
is exactly the class of bug a parity harness would have caught. The tooling
gates (clippy, rustfmt) are red, so convention compliance currently rests on
reviewer diligence. And the hand-mirrored wasm contracts (engine.ts surface,
SAMPLE_STRIDE offsets) are guarded only by a stride assertion, which cannot
catch a field reordering that keeps the width.

Missing. CI does not run clippy or rustfmt; the fixture pipeline is dead weight;
the docs have three live inaccuracies (conventions.md, the D4 compatibility
claim, the e_ct comment); and the avoidance demonstration reports no number on
screen for the behavior it exists to show.

Verdict: PASS overall for its purpose, with the findings above as the work
needed to keep it that way. None of the findings affect a flown trajectory
today; the two High items are the ones to clear before the next phase builds on
the current baseline.

### Recommended next work items

1. Wrap contract and parity hardening (new work). Fix `wrap_to_pi`, port the
   Python wrap tests to `math.rs`, and stand up the Python-vs-Rust parity suite
   over the six duplicated layers by repurposing `fixtures/live_fixtures.json`.
   This closes the Medium duplication finding and would have caught the High one.
2. Tooling gates green (new work). Clear the clippy deny error and warnings,
   run `cargo fmt`, add clippy and fmt jobs to `.github/workflows/ci.yml`.
3. Phase 5c, generated TypeScript type mirrors (existing unstarted phase). Use
   ts-rs to generate `src/types/flight.ts` and `src/live/types.ts` from
   `core/src/types.rs`, retiring the hand mirrors and the e_ct comment class of
   drift along with them. The retired fixture generation from item 1 belongs
   here as generated artifacts.
4. Small docs corrections (new work). conventions.md rewrite, the D4 validator
   claim versus export_json.py behavior, README "cascaded PD", and the
   export_json docstrings.
5. Phase 8, mission authoring (existing unstarted phase). Once 1 and 3 land,
   authoring waypoints, wind, and faults in the browser is the next feature
   phase; fold in surfacing `avoidance_activations` and the core-reported
   clearance on the run tiles as part of the authored-scenario feedback.

## Full security review (2026-10-18)

Whole-project security review against the stated threat model: a client-side
teaching and simulation tool with untrusted scenario JSON, live-run config,
imported replay documents, and Python API misuse as the adversary's inputs, no
server and no auth. Areas reviewed: input validation and untrusted data, the
web surface, native boundary safety (wasm and PyO3), secrets and data hygiene,
browser denial of service, and CI/workflow security. The 2026-10-18 full
codebase review above is treated as read; its non-security findings are not
repeated except where they carry security weight.

### Verification performed

| Check | Result |
|-------|--------|
| Grep: `unsafe` in `**/*.rs` | No matches anywhere (including bindings) |
| Grep: `dangerouslySetInnerHTML`, `eval(`, `new Function`, `innerHTML`, `document.write`, `postMessage`, `Worker(` | No matches in source (only prose in this file) |
| Grep: `system(`, `popen`, `exec(`, `subprocess` with `shell=`, `os.system`, `yaml.load`, `pickle` | Only `subprocess.run` with list args and fixed commands in `python/sims/run_all.py:273,294` |
| Grep: secrets (`api_key`, `token`, `password`, private key blocks, `AKIA`, `ghp_`, `sk-`) | No committed credentials; only workflow prose about Pages scopes |
| Grep: absolute developer paths (`/Users/`, `/home/`, `C:\`), `.env` files | None in tracked files |
| `npm audit --omit=dev`, `cargo audit` | Binaries not available in this review environment. Substituted direct OSV advisory queries (api.osv.dev) for every direct dependency at its locked version: npm (react 19.2.8, react-dom 19.2.8, three 0.170.0, zustand 5.0.15, vite 6.4.4, vitest 5.0.4, typescript 5.7.3, tailwindcss 3.4.19, @react-three/fiber 9.8.1, drei 10.7.9, postprocessing 3.2.0, plugin-react 4.7.0), cargo (nalgebra 0.33.3, serde 1.0.229, serde_json 1.0.154, wasm-bindgen 0.2.129, pyo3 0.29.3, libm 0.2.16), PyPI (numpy 2.5.4, scipy 1.18.1, matplotlib 3.11.2, pymavlink 2.4.50, pytest 9.1.1). Every candidate advisory was range-checked against the pinned version: zero affected. `package-lock.json`, `Cargo.lock`, and `uv.lock` are all committed |
| Lockfiles and install commands | `npm ci` in CI, `uv sync` against `uv.lock`, `Cargo.lock` committed; no curl or wget in any script |

### Findings

| Severity | Area | File:Line | Issue | Exploitable in threat model? | Remediation |
|----------|------|-----------|-------|------------------------------|-------------|
| Medium | Input validation | core/src/sim.rs:121-127, core/src/scenario.rs:141-146, wasm/src/lib.rs:117 | Validation accepts non-finite numbers. `duration_s` passes `!(v > 0.0)` when it is `+inf`, and serde_json parses `1e999` as `inf`, so `"duration_s": 1e999` in scenario JSON reaches `total_steps = ((duration_s / DT).round() as usize).max(1)`, a saturating cast to `usize::MAX`: an effectively endless run whose histories grow without bound. `takeoff_altitude`, `gps_*_noise`, `wind_sigma`, `wind_steady_ned`, `cruise_hold_s`, waypoint positions and radii, and obstacle centers/radii are never finiteness-checked at all. No panic or UB follows (safe Rust; NaN flows to output and the UI guards non-finite readouts), but a crafted scenario is an unbounded CPU and memory sink. | Theoretical today (the only JSON reaching this parser is the committed `public/data/live/configs.json`), becomes directly exploitable when Phase 8 authoring or replay import feeds user JSON to `ScenarioConfig::from_json` | In `SimConfig::validate` and `ScenarioConfig::validate`, require `is_finite()` on every numeric field plus sane upper bounds (for example `duration_s <= 3600.0`, wind and noise magnitudes capped), and bound `waypoints.len()` and `obstacles.len()`. Reject, do not clamp, matching the avoidance validator's posture |
| Medium | Input validation | core/src/avoidance.rs:112-123 | `AvoidanceConfig::validate` rejects negatives with `< 0.0` comparisons, which pass `NaN` and `inf`. A `"avoidance": {"clearance_margin_m": 1e999}` block yields infinite detour standoffs and NaN waypoints downstream. | Theoretical (same gate as above: user-authored configs) | Use `!(v.is_finite() && v >= 0.0)` style checks for margin, buffer, and lookahead |
| Medium | Input validation | src/live/readout.ts:101-102, src/views/RunView.tsx:96-99 | `bufferCapacity = ceil(duration_s / DT_CONTROL) + 64` sizes a `Float64Array(capacity * 36)` plus seven more typed arrays with no cap. A huge or non-finite `duration_s` in `configs.json` raises `RangeError` inside a `useMemo`, which is a render-phase throw (white screen), or attempts a multi-gigabyte allocation. | Theoretical today (committed config), real once configs are user-supplied | Clamp capacity to a hard ceiling (for example 24 hours of samples) and fail the run with a visible message instead of allocating; validate `duration_s` before constructing the buffer |
| Medium | Input validation | src/data/live.ts:16, src/store/useFlightStore.ts:111-114 | Fetched JSON is cast to `LiveConfig` / `FlightData` with no runtime validation. Parallel array lengths, numeric finiteness, and array sizes are all trusted. The only normalization is the `obstacles` fallback at useFlightStore.ts:114. | Theoretical (same-origin committed assets; no import path exists) | Add a small TS validator (or generate one with the Phase 5c ts-rs mirrors) before any document import or authoring feature lands; the Python `validate_flight_json` is the model |
| Medium | CI | .github/workflows/ci.yml:13-16 | The CI workflow declares no `permissions:` block, so jobs run with the repository default token permissions (read/write on older repos). `deploy.yml:39-42` gets this right. | Not attacker-exploitable by itself, but it is the standard blast-radius amplifier if any CI step is compromised | Add `permissions: contents: read` at the top of ci.yml |
| Low | Input validation | wasm/src/lib.rs:152-160, 266-268 | `advance(steps: usize)` and `samples_from(from: usize)` accept wasm-bindgen coerced unsigned values; a negative JavaScript number wraps to roughly 4e9, so `advance(-1)` behaves like `run_to_end` in one call (multi-second tab freeze) and `samples_from(-1)` returns the whole history. The page itself always passes guarded values (RunView.tsx:205), so this is surface hygiene, not a live bug. | Not reachable from the app; reachable to any future consumer of the wasm module | Take `f64` and reject non-finite or negative arguments, or clamp to `total_steps` / `sample_count` explicitly |
| Low | Input validation | core/src/ekf.rs:307-309 | `try_inverse().expect(...)` is the one production panic reachable in principle from configuration (zero GPS noise plus a numerically singular innovation covariance). In wasm a panic aborts the module instance (the run dies, the page survives); in PyO3 it surfaces as `PanicException`. The invariant `expect` sites at sim.rs:1048,1103,1143 and motors.rs:62 are of the same class. | Theoretical (requires extreme config) | Return `false` (treat as a gated fix) as the codebase review also recommends; a flight core should be panic-free under every configuration |
| Low | Native boundary | python/talon/mavlink_sil.py:1036-1038, 1051-1071 | The command socket binds `telemetry_host` (default `127.0.0.1`, good) but any local process can send a COMMAND_LONG that queues `arm` / `takeoff` / `rtl` / `mission_start` events with no authentication, and `int(fields["command"])` at line 1069 reads a dict key outside the `try`, so a parseable COMMAND_LONG without a `command` field raises `KeyError` through `_drain_commands` and kills the sim loop thread. | Loopback-only, sim-affecting only (teaching harness). Local process attacker | Read the field with `fields.get("command")` inside the guarded region and ignore unknowns; document that the UDP surface is an unauthenticated teaching convenience |
| Low | Native boundary | python/talon/mavlink_sil.py:527-535 | `run(duration_s)` validates `> 0` only, so `run(float('inf'))` from Python is a never-terminating loop (API misuse hang). The Rust `SimRunner::run` (core/src/sim.rs:678-681) has the same edge. | Python API misuse (stated in threat model) | Reject non-finite durations in both |
| Low | Input validation | python/talon/export_json.py:279-381 | `validate_flight_json` checks types and shapes but not numeric finiteness: NaN passes `isinstance(radius, (int, float))` and `radius <= 0` is false for NaN, so `radius_m: NaN` (via `json.loads` of a literal `NaN`, which Python accepts) and `1e999` values in the series arrays are accepted. The module has `_check_finite` (line 124) but the validator never calls it. This compounds the earlier Medium finding that `obstacles` is required here while D4 claims compatibility. | Exploitable wherever the validator is the trust gate for imported documents (today, the Python harness) | Run a finiteness pass over every numeric leaf (reuse `_check_finite`), reject `NaN`/`inf` radii explicitly, and decide the `obstacles` optional-vs-required question |
| Low | Web surface | index.html:11-16 | The page loads Google Fonts CSS at runtime with no Subresource Integrity and the app ships no Content-Security-Policy. It is the site's only third-party request; a fonts provider compromise is a (small) script-injection and privacy exposure. GitHub Pages cannot set headers, so CSP would have to be a meta tag. | Inherent third-party exposure; low impact | Self-host IBM Plex Mono and Inter (they are already the only external assets), or add SRI where possible plus a restrictive CSP meta tag |
| Low | Supply chain | .github/workflows/ci.yml:81-93 | The wasm drift check compares the generated JS and declarations but deliberately excludes `public/wasm-rust/talon_core_bg.wasm`, which is the binary the site actually executes. A tampered committed `.wasm` would pass CI. | Requires a compromised commit (at which point CI is untrusted anyway) | Record a hash of the built `.wasm` at build time and have the deploy workflow compare it to a freshly built one, or accept and document the gap as done here |
| Low | CI | .github/workflows/*.yml | Actions are pinned by mutable tags, not commit SHA (`actions/checkout@v4`, `actions/setup-python@v5`, `actions/setup-node@v4`, `actions/configure-pages@v5`, `actions/upload-pages-artifact@v3`, `actions/deploy-pages@v4`, `jetli/wasm-pack-action@v0.4.0`, and `dtolnay/rust-toolchain@stable`, which floats to the newest toolchain). `deploy.yml:78` also interpolates `${{ steps.pages.outputs.base_path }}` directly into a `run:` block instead of passing it through `env:`. | Supply-chain only; the base_path value is derived from the repo name by a trusted action, and no `github.event.*` value reaches any `run:` block, so there is no PR-title script-injection path | SHA-pin the actions (with dependabot to keep them fresh) and move expression interpolation to `env:` |

No Critical findings. No High findings.

### Per-area verdicts

| Area | Verdict | Summary |
|------|---------|---------|
| 1. Input validation and untrusted data | PASS with findings | Typed errors everywhere, no panics on ordinary bad input, `validate()` covers duration and takeoff sign plus the avoidance knobs, and the mission layer even guards NaN positions (mission.rs:107-112). The gap is finiteness and magnitude: `inf`/`NaN` pass every numeric check in the Rust core, the TS loaders cast without validating, and the Python validator checks shapes but not values. All of it is theoretical while the only inputs are committed assets, and all of it becomes real at Phase 8 or at any document-import feature |
| 2. Web surface (XSS, fetch, supply chain) | PASS | No dynamic code evaluation, no HTML injection sinks, no user-controlled string reaches a fetch, import, or `src`. React text nodes and canvas sprite labels only. The wasm loader's dynamic `import()` takes a build-time base path, which is safe today and should stay that way. Dependency set is small, mainstream, lockfile-pinned, and clean of known advisories at the pinned versions |
| 3. Native boundary safety | PASS | Zero `unsafe` in the workspace, PyO3 generates the FFI shims, `vec3_from` is bounds-checked with a counter (convert.rs:24-40), no Python-controlled indexing exists, the sample stream is length-bounded and `?? fallback` guarded on the JS side (stream.ts:99-159), the stride contract is runtime-checked against the loaded build (RunView.tsx:126-133), and `LiveBuffer.append` truncates to remaining capacity. Memory unsafety is not reachable from any input; the only crash vectors are the invariant panics listed above, which kill a run, never memory |
| 4. Secrets and data hygiene | PASS | No credentials, tokens, keys, `.env` files, or absolute developer paths in tracked files. The one network surface is the loopback MAVLink UDP pair (127.0.0.1 by default). Logs print numbers, not secrets |
| 5. Browser denial of service | PASS for today's inputs | Work per frame is capped (`MAX_STEPS_PER_FRAME = 4000`, RunView.tsx:28; pacer clamps frame deltas; the recorder is preallocated; chart decimation is O(n) with a bounded point budget; sessions and GPU resources are freed). Every unbounded growth vector is keyed to config-supplied durations and counts, so the DoS story is exactly the input-validation story in Area 1 |
| 6. CI/workflow security | PASS with findings | deploy.yml has correct least-privilege permissions and a concurrency guard; ci.yml needs its `permissions:` block; actions should move to SHA pins; no script injection is possible from PR titles or commit messages because no `github.event.*` value is interpolated into `run:` |

### Inherent to a client-side teaching tool versus real bugs

Inherent (accepted, no fix possible or worthwhile):

- There is no server, so there is no authentication, authorization, session, or
  server-side injection surface to protect. The attacker model is "malicious
  data fed to a willing victim's own tab", and the worst achievable outcome for
  the shipped app is the victim's own tab hanging or showing wrong numbers.
- A user can always freeze their own tab by running long missions at high rate;
  the per-frame caps bound each frame but cannot and should not stop the user
  from simulating.
- Numeric drift, NaN rendering as `n/a`, and wasm memory growth proportional to
  simulated time are properties of the domain, not defects.
- The committed `public/wasm-rust/` bundle and `public/data/` JSON are trusted
  inputs by construction (they are the deployment); defending against a
  malicious commit is a git-protection problem, not an application one.

Real bugs or gaps worth fixing regardless of threat model:

- The missing finiteness and magnitude validation in `SimConfig::validate`,
  `ScenarioConfig::validate`, `AvoidanceConfig::validate`, and
  `validate_flight_json`. `1e999` is a JSON-reachable `inf` today, and the
  validator's job is to say no to it.
- The `RangeError` white-screen path from an unbounded `bufferCapacity`.
- The missing `permissions:` block in ci.yml.
- The `KeyError` in `_drain_commands` on a field-less COMMAND_LONG.
- The unauthenticated loopback command socket should at least be documented as
  such in the module docstring.

### Security posture verdict

PASS. The project has a genuinely small attack surface (no server, no auth, no
dynamic code execution, no file or process access on the web side), zero unsafe
code across three Rust crates, correct bounds discipline at both the wasm and
PyO3 boundaries, a dependency set that is small, locked, and clean of known
advisories, and least-privilege CI with no script-injection path. Nothing found
is exploitable against the app as it stands. The Medium findings are all the
same class: validation that trusts its numbers to be finite and sane, which is
harmless while inputs are committed assets and becomes the whole ballgame the
moment mission authoring (Phase 8) or replay import turns user JSON into
`ScenarioConfig` or `FlightData`. Land the validation hardening before, not
after, that phase.

### Notes for other agents

- Recurring pattern to avoid: sign-based validation (`v < 0.0`, `v > 0.0`) in
  this codebase silently admits `NaN` and `inf`. New numeric knobs should be
  validated with `v.is_finite() && v > 0.0` (or the range check plus
  `is_finite()`), in Rust and in the Python validator alike.
- The trust boundary is currently "committed JSON", and three separate loaders
  (`ScenarioConfig::from_json`, `loadLiveConfigs`, `loadScenario`) cast or parse
  without value-level checks. When Phase 8 or import work starts, those three
  seams are where the validator belongs.

### Addendum (2026-10-18): TypeScript deep dive incorporated

The web-app deep dive completed after the main pass and was independently
verified against the committed wasm artifacts before inclusion. It changes two
verdicts and adds findings.

Verdict changes:

- "wasm boundary contract": PASS becomes FAIL. The method and getter names all
  match, but one getter type does not (the `bigint` finding below).
- "TS type mirrors vs schema": the FAIL now covers two items (the `e_ct`
  comment and `LiveConfig.battery_initial_soc`).

Additional findings:

| Severity | File:Line | Issue | Fix |
|----------|-----------|-------|-----|
| High | src/live/engine.ts:41-45 vs public/wasm-rust/talon_core.d.ts:114,118,122 and talon_core.js:121-139 | `gps_accepted`, `gps_gated`, `gps_skipped` are `u64` in Rust (wasm/src/lib.rs:284-299), which wasm-bindgen maps to JS `bigint`; the hand-declared surface claims `number`. The FakeSession in src/live/sim.test.ts returns numbers, so the typecheck and the tests pass while the real module returns BigInt. It survives today only because every consumer stringifies (TelemetryRail.tsx:178); any arithmetic mixing (`gps[0] / 2`) throws "Cannot mix BigInt and other types" at runtime, and `gpsCounts()` in src/live/sim.ts:176-178 already returns bigints typed as numbers. | Declare the three getters as `bigint` in CoreSession, convert at the seam (`gpsCounts(): [number, number, number]` wrapping `Number(...)`), and update FakeSession to return bigints so the fake cannot mask the drift again |
| Medium | src/live/types.ts:43-63 vs core/src/scenario.rs:85-86 | LiveConfig lacks `battery_initial_soc` (Rust `ScenarioConfig` carries it with default 1.0; the exporter does not emit it). Benign today, but Phase 8 config authoring built on this type would silently drop the battery knob. | Add `battery_initial_soc?: number` to LiveConfig and emit it from export_live_configs.py |
| Medium | src/live/types.ts:1-8,12,26 | Header still claims the config mirrors `talon.mavlink_sil.SILConfig`; since Phase 5b the source of truth is `core/src/scenario.rs`. | Rewrite against the Rust types |
| Medium | src/live/stream.ts:93-99 | `decodeSample` is documented "Kept free of allocation" but allocates 12 to 13 tuples per call, and `poseAt` adds more; the claim contradicts the design goal and the LiveBuffer "no allocation" note (stream.ts:203). | Mutate preallocated scratch vectors in place, or soften both comments |
| Low | src/live/sim.ts:231-245 vs src/utils/clearance.ts:58-69 | Local `minClearance` duplicates `minSurfaceClearance` with different empty-input semantics (null vs Infinity); pure logic is meant to live once in src/utils. | Delete the copy, map Infinity to null at the summary assembly (this folds into the earlier summary finding) |
| Low | src/live/sim.ts:94-96,104-107,140-144; src/live/engine.ts:78-79; src/node-shims.d.ts:11-13 | Dead or test-only surface: `LiveSim.advance`, `runToCompletion`, `groundSpeed`; `CoreModule.core_version`/`run_scenario` unused by the app; an unused `node:module` ambient shim. | Remove, or annotate as deliberate test/batch seams |
| Low | scripts/smoke_live_wasm.mjs:122 | `phases[phases.length - 1][1]` throws on an empty timeline instead of failing a check. | Guard the index and check for null |
| Low | src/utils/format.ts:7 | `'--'` as the non-finite placeholder is the one double-hyphen spot in src/ (instrument placeholder, not prose, but inconsistent with `fmt()`'s `'n/a'` elsewhere). | Unify the unavailable sentinel across the app |
| Low | vitest.config.ts:11-12 | Node environment, `*.test.ts` only: no component or DOM test tier exists, so the RunView loop wiring and effect cleanup are guarded only by the typecheck. | Add a `*.dom.test.tsx` tier (jsdom or happy-dom) when the loop logic next changes |
| Low | src/components/live/LiveField.tsx:133-134,209,219; src/components/scene/ObstacleField.tsx:48-49,122,132 | `fillRefs`/`wireRefs` are never trimmed when a config with fewer obstacles replaces one with more; stale mesh refs live until unmount. | Clear the ref arrays in an effect keyed on the obstacle list |
| Low | src/live/types.ts:33-41 vs core/src/avoidance.rs:86-96 | TS `AvoidanceConfig.enabled` is required; Rust's `#[serde(default)]` accepts a block without it. The TS type rejects a shape the core accepts. | Make `enabled?: boolean` or document the asymmetry |

Verified PASS items from the same deep dive, worth recording: all 13 sample
offsets and the stride match the core on both sides and are pinned by tests;
`indexAt`/`poseAt` semantics are correct at both clamps; rAF, fetch-cancellation,
observer, and Three.js disposal hygiene is complete; hooks rules hold in every
component checked; there is no dynamic code execution or unsanitized HTML in
src/; `strict` plus `noUncheckedIndexedAccess` (and unused-locals, unused-params,
fallthrough) are genuinely enforced; `scripts/copy_data.sh` is sound and its
scenario list matches the store's closed union.

The High finding joins work item 1 above (contract hardening), and the
`battery_initial_soc` gap joins item 3 (Phase 5c generated mirrors) as the
reason the mirrors should be generated rather than hand-declared. Neither new
finding changes the overall verdict: PASS for the project's purpose, with the
High items cleared before Phase 8 builds on these contracts.

### Addendum (2026-10-18): Python harness deep dive incorporated

The Python deep dive completed last and was spot-verified against the source
(`mavlink_sil.py:539, 1207-1208`, and the `<= 0.0` validation sites) before
inclusion. It independently reproduces the `wrap_to_pi` finding and adds the
following.

| Severity | File:Line | Issue | Fix |
|----------|-----------|-------|-----|
| Medium | python/talon/mavlink_sil.py:1207-1208 vs core/src/sim.rs:1252 | `waypoints_reached` metric drift: Python counts `active_index + 1` unconditionally, Rust adds one only when the mission is complete. For an incomplete mission the Python number over-reports by one, and it flows into metrics, sil_reference.json, and scenario output (scenario_gps_outage.py:130, scenario_obstacle_field.py:113). | Match the Rust rule (`active_index + (1 if complete else 0)`) or rename the metric, and update test_integration.py:149 |
| Medium | python/talon/mavlink_sil.py:957-969, 185, 1257; python/sims/scenario_obstacle_field.py:21-22 | The Python SIL loop has no reactive avoidance: `_control_tick` calls `guidance.compute_mission` directly, `SILConfig.obstacles` is loop-inert (forwarded to the result only), and the bindings expose the five offline geometry functions but not `ReactiveAvoidance`. The `avoidance` live config can therefore only be flown in the browser, and scenario_obstacle_field.py's claim that "the planner and the reactive law the autopilot flies in the loop are one implementation" is true only of the planner. | Either expose `ReactiveAvoidance` through bindings and wire it into `_control_tick`, or correct the docstring and record in decisions.md that the Python harness flies pre-planned detours only |
| Medium | python/talon/mavlink_sil.py:539-555 | `_open_sockets()` runs before the `try:`; a partial failure leaks the sockets already created, `finally` never runs, and the exception is swallowed by the `start_async` daemon thread. | Move `_open_sockets()` inside the `try` and surface thread exceptions through an error flag |
| Medium | python/talon/state_machine.py:216, motors.py:85, battery.py:41-46, types.py:99, mavlink_sil.py:196-204, sensors.py:90,177 | NaN validation hole class: `if x <= 0.0` admits NaN (NaN compares False), so `takeoff_altitude`, `time_const`, `capacity_mah`, `hover_thrust_n`, `radius_m`, `dt`, and `telemetry_rate_hz` accept NaN and poison a run silently. Rust rejects NaN via `!(x > 0.0)` (state_machine.rs:204 with a NaN test at :329, motors.rs:127, battery.rs:86-91), which is also the pattern behind the clippy `neg_cmp_op_on_partial_ord` warnings. | Use `if not (x > 0.0)` in the Python copies and add NaN cases to test_state_machine.py:421 and test_battery.py:77 |
| Medium | python/talon/mission.py:98-99 vs core/src/mission.rs:112-114 | Non-finite position contract differs (Python raises ValueError, Rust returns early and keeps flying), and mission.rs:107-108 claims the guard exists because the comparison behaves "exactly as in the Python source", which is not what the Python source does. The Python guard is untested. | Pick one contract or state the divergence in both docstrings; add a NaN test to test_mission.py |
| Low | python/sims/scenario_waypoints.py:331, python/talon/animation.py:593 vs mavlink_sil.py:678-680, sim.rs:837-840 | Dryden `wind.step` is fed the 3-D speed norm in the Phase 4 scenarios and animation but the horizontal NE norm in the SIL and the core; same variable name, two definitions. | Standardize on the horizontal norm everywhere |
| Low | python/talon/battery.py:36 vs core/src/battery.rs:52 | `hover_thrust_n = 14.715` hardcoded where Rust derives `MASS * GRAVITY`. | Derive from `talon.constants` |
| Low | python/talon/{motors,battery,sensors}.py vs core/src | `dt < 0` policy drift (Python raises, Rust clamps), expanding the earlier API-edge finding. | Document one policy per layer |
| Low | python/talon/sensors.py:162-167, 202 | `true_state: object` on parameters whose fields are accessed; convention requires real type hints. | Type as `State` or a Protocol |
| Low | python/talon/avoidance.py:47-49, 38 | `_vec` raises bare `IndexError` on a 2-element input instead of `ValueError`; missing blank line after imports. | Validate the shape, raise `ValueError` |
| Low | python/talon/mavlink_sil.py:958, 972; python/sims/scenario_waypoints.py:198, scenario_rtl.py:211 | Runtime `assert` for invariants, stripped under `python -O`. | Raise explicitly |
| Low | python/talon/mavlink_codec.py:246, mavlink_sil.py:246, monte_carlo.py:102, run_all.py:215, export_live_configs.py:77-98, tests/ | Type-hint gaps (bare `dict`, untyped params, missing `-> None`). | Annotate; rename `test_avoidance`'s `id` param |
| Low | python/sims/export_live_configs.py:350, 292 | Dead `mission_wps and` truthiness expression; `final_quat` written unrounded in a fixture meant for byte-stable parity. | Drop the expression; round like the rest |
| Low | python/sims/run_all.py:193, 1-14, 357, 396 | `PYTEST_EXPECTED = 139` stale (suite is 186); docstring says six scenarios while seven run; usage line names the wrong path; `scenario_obstacle_field` is fully re-simulated three times per driver run. | Update constants and docstring, run the scenario once |
| Low | python/talon/animation.py:410-624 (with :86-105), plotting.py:621 vs animation.py:114 | `_run_waypoints_with_capture` duplicates the waypoint scenario loop with 12 hand-mirrored constants (the largest duplication in the harness), and the phase-color lookup exists twice. | Move the shared loop into `talon` and share one lookup helper |
| Low | python/tests/test_mavlink_sil.py:167, 226, 264; test_monte_carlo.py:36-43; mavlink_sil.py:643, 652 | Fixed UDP ports collide with parallel runs; the Monte Carlo smoke test asserts mostly isinstance; `_history_gps_times` is appended outside the lock the module docstring promises. | Bind port 0, add a numeric bound, take the lock or fix the docstring |

The fixtures finding stands confirmed with stronger evidence: both
`fixtures/live_fixtures.json` and `fixtures/sil_reference.json` are generated
and read by nothing, the vitest consumers named in the exporter docstrings were
deleted in Phase 5b, and the "browser-run integration test" `sil_reference`
claims does not exist. It remains Medium (folded into work item 1).

Test quality is good overall: quantitative thresholds, an independent
pure-Python RK4 reference in test_dynamics.py:17-124, `critical_fault`
parametrized over all eight phases, full-stack SIL integration asserts. The
gaps are the untested non-finite mission guard, no NaN constructor cases, and
no fixture pinning `chi_d` below `-pi`.

Recurring pattern for future work: every cross-language pair in this repo
should be diffed with NaN and negative-input behavior in mind (the Python
copies validate with `x <= 0.0` and raise on bad `dt` where the Rust originals
use `!(x > 0.0)` and clamp), and parity fixtures must include at least one case
in each function's wrap and branch corner; the existing guidance fixtures all
sit in the benign quadrant, which is why the `wrap_to_pi` defect survived both
suites.

Revised tallies: no Critical; 4 High (`wrap_to_pi`, clippy deny-level error,
rustfmt non-compliance, wasm `bigint` getter drift); 11 Medium; roughly 30 Low
(mostly hygiene and documentation). Overall verdict unchanged: PASS for the
stated purpose, with the four High items as the gate before Phase 8.

### Phase 1 review (2026-10-18)

Scope: `core/src/math.rs` only. The change fixes `wrap_to_pi` for inputs below
`-pi` (from `fmod(theta + PI, TAU) - PI` to `(theta + PI).rem_euclid(TAU) - PI`)
and adds the file's first test module, ten tests. This closes the High
`wrap_to_pi` finding from the full codebase review above.

Verdict: PASS. The new formula is bit-identical to Python `wrap_angle`
(guidance.py:110-112), the tests fail on the old implementation, and no other
file changed.

- PASS: correctness and parity. `rem_euclid` and CPython float `%` run the same
  fmod-then-adjust algorithm, so the two are bit-identical across 8020 sampled
  inputs (the four pinned cases from test_guidance.py:19-22, large magnitudes
  such as 1e300, signed zero, NaN, and both infinities, which yield NaN on both
  sides). Exact boundaries verified bit for bit: `wrap_to_pi(PI) == -PI` and
  `wrap_to_pi(-PI) == -PI` (math.rs:127-131); NaN propagates (math.rs:133-136).
- PASS: the doc comment makes only true claims now. The stale TypeScript parity
  claim is removed, and the remaining claims (one-to-one Python match, half-open
  upper bound, NaN yields NaN) all hold.
- PASS: test quality. Three of the ten tests fail on the old form:
  `wrap_matches_python_pinned_cases` (2 of its 4 asserts),
  `wrap_recovers_large_negative_input`, and `sweep_stays_in_half_open_interval`
  (943 of 2514 grid points violate `[-pi, pi)` against the old `fmod` form). The
  sweep asserts the range invariant instead of restating the formula, the pinned
  values come from the Python tests, and the 1e-12 absolute tolerance is
  appropriate for exact fmod arithmetic.
- PASS: regression safety. `git status` shows `core/src/math.rs` as the only
  modified file (one unrelated untracked `eagle-shield.svg` aside). The only
  consumer is `chi_d` (guidance.rs:209), which feeds `v_des` through `cos` and
  `sin` (guidance.rs:215-216); those are 2*pi-periodic and the old and new forms
  differ by exactly `TAU` wherever both are finite, so flight output is
  unchanged and only the diagnostics value becomes correct. `cargo test -p
  talon-core` is 220 passed, 0 failed, which is the prior 210 plus the 10 new
  tests.
- PASS: conventions. rustfmt reports no diff for `core/src/math.rs` (the 78
  pre-existing fmt diffs across 15 other files are untouched); no em dashes or
  dash equivalents; 4-space indent and the shared `#[cfg(test)] mod tests` style
  with `use super::*;`.

| Severity | File:Line | Issue | Fix |
|----------|-----------|-------|-----|
| Low | core/src/math.rs:50 | For the single input `nextafter(-pi, -inf)` the negative adjustment rounds to exactly `TAU`, so the result is `+pi`, just outside `[-pi, pi)`. Python `wrap_angle` returns the same `+pi`, so parity holds; it is a shared IEEE rounding artifact of the floored-mod algorithm and unreachable from guidance (`chi_q + nu` stays well inside one revolution). | Accept as shared documented behavior, or map `w == PI` to `-PI` if the strict contract is ever needed |
| Low | core/src/math.rs:45-47 | "guidance output is identical in Rust and Python" is exact for the wrap itself, but downstream `atan2`/`sin`/`cos` come from different libms (Rust vendors `libm`, Python uses the platform one), so ULP-level differences in `chi_q` remain possible. | Soften to "the wrap is identical" or note the ULP caveat |

Recurring pattern: parity fixtures must keep at least one case in each
function's wrap and branch corner, per the Python harness deep dive above; the
Rust side of the wrap corner is now pinned by `wrap_matches_python_pinned_cases`
and `sweep_stays_in_half_open_interval`.

### Phase 2 review (2026-10-18)

Scope: Phase 2b cleanup, uncommitted at review time on top of `1b63d4e`
(rustfmt): clippy deny-level cleanup across `core/src/{guidance,battery,motors,
scenario,sim,state_machine,dynamics,math}.rs` and
`bindings/src/{avoidance,estimation,types}.rs`, CI gates in
`.github/workflows/ci.yml`, and the TypeScript wasm contract fix in
`src/live/{engine,sim,sim.test}.ts` plus the new
`src/live/engine.contract.test.ts`. Closes the High clippy deny-level, rustfmt
non-compliance, and wasm `bigint` getter drift findings from the full codebase
review above.

Verdict: PASS. All seven verification commands are green, every rewrite is
behavior preserving, and the contract test was mutation-proven to fail on name
and type drift. No blocking issues; three Low notes below.

- PASS: NaN semantics of the six `neg_cmp_op_on_partial_ord` rewrites. Every
  site (core/src/battery.rs:88, core/src/battery.rs:91, core/src/motors.rs:129,
  core/src/scenario.rs:144, core/src/sim.rs:124,
  core/src/state_machine.rs:206) reads `x <= 0.0 || x.is_nan()`, which equals
  the original `!(x > 0.0)` for every input class, confirmed case by case:
  NaN is true on both sides (the new form via `is_nan`, the old because
  `NaN > 0.0` is false), negative and both zeros are true on both sides, and
  positive values and infinities are false on both sides. No rewrite changed
  behavior. Each site carries a comment stating the NaN intent, which matters
  because a bare `x <= 0.0` would have silently accepted NaN.
- PASS: `BEHIND_S_THRESHOLD` (core/src/guidance.rs:55) is bit-identical to the
  old `-0.7071` literal. Both `-7071.0 / 10_000.0` and the decimal literal are
  correctly rounded values of the same rational (negative 7071 over 10000;
  both division operands are exactly representable, and IEEE 754 rounds both
  the literal and the division to nearest even), so all 64 bits agree.
  Emulation confirms `Object.is` equality, and the trigger comparison
  `s < BEHIND_S_THRESHOLD` (core/src/guidance.rs:203) matches Python
  `_BEHIND_S_THRESHOLD = -0.7071` (python/talon/guidance.py:94) across every
  boundary sample, so parity holds. The `approx_constant` workaround is
  legitimate and well commented.
- PASS: the `collapsible_if` merges in `SimRunner::handle_events`
  (core/src/sim.rs:968-1028, eight merges) preserve evaluation order and short
  circuit behavior. The state-mutating `state_machine.request(...)`, which
  logs every evaluation to `event_log` accepted and rejected alike, remains the
  rightmost `&&` operand at every site, so it runs exactly once and only when
  all guards pass, exactly as the nested form did, and `note_phase_change()`
  still runs iff `request` returns true. No logging, phase, or ordering change.
- PASS: the remaining clippy fixes are behavior preserving and the public API
  is unchanged where it matters. `needless_update` (core/src/dynamics.rs:147-156):
  dropping `..Default::default()` with every field listed is a no-op (the crate
  compiles, so no field is missing; the remaining `..Default::default()` at
  dynamics.rs:68 is a different literal that needs it). `field_reassign_with_default`
  (core/src/dynamics.rs:243, core/src/dynamics.rs:291, core/src/sim.rs:433):
  struct literal plus `..State::default()` yields the same final state as
  mutate after default. `needless_range_loop` (core/src/motors.rs:156): the
  `iter_mut().zip(commanded)` loop performs the same per-element arithmetic in
  the same 0 to 4 order over two length-4 arrays, and the math.rs:120
  `(-PI..PI).contains(&w)` predicate is identical to `w >= -PI && w < PI`.
  `type_complexity`: `DetourRoute` (bindings/src/avoidance.rs:138) is a private
  alias, so the PyO3 surface is unchanged, and `ScheduledAction`
  (core/src/sim.rs:298) is an additive public alias with `SimRunner::schedule`
  type-identical, so callers still compile. Derivable `Default`
  (bindings/src/estimation.rs:11, bindings/src/estimation.rs:169):
  `#[derive(Default)]` over manual impls delegating to the inner defaults is
  the same trait behavior. The single `#[allow(clippy::too_many_arguments)]`
  (bindings/src/types.rs:259) is justified by the keyword-argument Python API
  it mirrors and is the only `allow` in the tree.
- PASS: CI gates (.github/workflows/ci.yml). The fmt step (line 51) and the
  clippy step (line 58) sit after `uv sync` (line 41) and after the toolchain
  install carrying `components: rustfmt, clippy` (line 46), so both tools exist
  and `.cargo/config.toml` (PYO3_PYTHON at `.venv/bin/python`) is satisfied
  before the workspace compiles. Workflow-level `permissions: contents: read`
  (lines 15-16) suffices for all three jobs: checkout, builds, tests, and the
  local `git diff --exit-code` all need read access only. The YAML is well
  formed. The gate fails on drift: `cargo fmt --all -- --check` exits non-zero
  on any formatting diff and `cargo clippy --workspace --all-targets -- -D
  warnings` denies every warning, both under `bash -e`.
- PASS: TypeScript contract. `CoreSession` now matches
  `public/wasm-rust/talon_core.d.ts` exactly for the GPS getters
  (`readonly gps_accepted: bigint` and siblings, d.ts:114-122) and every other
  member. `LiveSim.gpsCounts` (src/live/sim.ts:176) narrows with `Number(...)`
  to `[number, number, number]`, and the HUD consumers stay number-typed
  (src/live/readout.ts:77 destructures `gpsCounts`, src/views/RunView.tsx:506
  reads the JSON summary numbers, which are unrelated to the bigint boundary);
  sim.test.ts fakes bigint getters to match.
- PASS: `engine.contract.test.ts` genuinely fails on name or type drift.
  Proven by four temporary mutations, each reverted and hash-verified:
  generated-side type drift (`gps_accepted: bigint` to `number`) fails two
  tests with the mismatch message, generated-side name drift (`gps_gated`
  renamed) fails three tests as a missing member, mirror-side type drift fails
  `declares every member the app relies on...` with the mismatch message, and
  a mirror-side rename fails the same test as a missing member. Baseline is 5
  passed and the tree was byte-identical afterwards.
- PASS: verification, all from the repository root: `cargo clippy --workspace
  --all-targets -- -D warnings` clean, `cargo fmt --all -- --check` clean,
  `cargo test --workspace` 235 passed (220 talon-core, 3 talon-py, 12
  talon-wasm), `npm test` 137 passed across 11 files, `npm run build`
  typecheck plus production bundle green.

| Severity | File:Line | Issue | Fix |
|----------|-----------|-------|-----|
| Low | src/live/engine.contract.test.ts:98-133 | The `required` name pin list is asserted against the generated Session only; deleting a member from the CoreSession mirror escapes every assertion (test 1 iterates the mirror, tests 2 and 3 read the generated file). tsc still catches the fallout at the usage sites, so nothing ships broken. | Assert the `required` names against `coreSessionMembers(ENGINE)` as well |
| Low | core/src/sim.rs:650, core/src/sim.rs:689 | Pre-existing `duration_s <= 0.0` guards accept a NaN duration (`NaN <= 0.0` is false); untouched by this change and now inconsistent with the new `<= 0.0 \|\| is_nan()` guards. | Fold into a future hardening pass: append `\|\| duration_s.is_nan()` |
| Low | dist/assets (npm run build) | Non-fatal vite warning: the 1247.98 kB bundle exceeds the 1200 kB chunk suggestion. Pre-existing. | Code-split the live kernel and views when it starts to matter |

Recurring patterns: a clippy `neg_cmp_op_on_partial_ord` fix on a validation
guard must read `x <= 0.0 || x.is_nan()`, never bare `x <= 0.0`, or NaN slips
through; keep a comment saying so at the site. A `collapsible_if` merge is only
safe while the state-mutating call stays the last `&&` operand, which keeps
short circuit order equal to the nested form. A hand-mirrored wasm surface
needs a drift test that reads the generated declaration from disk; typechecking
the mirror alone cannot see the other side, and the pin list should be asserted
against both files.

### Phase 2 security review (2026-10-18)

Focused security review of the Phase 2b cleanup (same uncommitted change set as
the Phase 2 review above, on top of `1b63d4e`), scoped to the security weight of
five items: the CI workflow changes, the six NaN-preserving validation guard
rewrites, the bigint to Number narrowing in the live seam, the new contract
test's disk read, and any new secret, network, or dependency surface. The 2026
10-18 full security review and the Phase 2 review above are treated as read;
non-security findings and already-recorded findings are referenced, not
repeated. This phase closes the full security review's Medium CI finding
(missing `permissions:` block in ci.yml).

Overall verdict: PASS. All five items pass. No Critical or High findings; no
new Medium; two Low notes (one of which is a carried-over pre-existing entry).

| Severity | File:Line | Issue | Fix |
|----------|-----------|-------|-----|
| Low | core/src/scenario.rs:144, core/src/sim.rs:124 | The rewritten guards reject NaN but deliberately accept `+inf` (`inf <= 0.0` is false, `is_nan` is false), exactly as the old `!(x > 0.0)` form did. This is a faithful rewrite, not a regression, but it leaves the full security review's Medium finiteness finding (validation accepts `1e999`-parsed infinities) open, and the new `is_nan` comments could be read as "validation is now airtight" when it is not. | In the future hardening pass, tighten to `!(x.is_finite() && x > 0.0)` per the recurring pattern in the full security review |
| Low | src/live/engine.contract.test.ts:19-20 | `readFileSync` paths are relative, so the test depends on the process working directory being the repository root (it is for `npm test` and the CI `web` job). From any other cwd the read throws ENOENT and the suite fails closed, so this is an environment assumption, not a vulnerability. | Optionally anchor with `path.resolve(__dirname, '../..')` if the test is ever run from a non-root cwd |

Per-item results (PASS/FAIL):

1. PASS: CI workflow permissions (.github/workflows/ci.yml:15-16). The
   workflow-level `permissions: contents: read` constrains all three jobs
   (`core`, `wasm`, `web`): there is no job-level `permissions:` key anywhere in
   the file (grep: the only `permissions:` match is line 15), and GitHub applies
   the workflow-level block to every job that does not override it. Every step
   needs read only: `actions/checkout` (contents: read), toolchain and package
   installs (registry network, no token scope), `cargo fmt --all -- --check`
   (line 51) and `cargo clippy --workspace --all-targets -- -D warnings`
   (line 58) as pure local read-only compiler passes, `cargo test`, `npm ci`,
   `npm test`, `npm run build`, and the local `git diff --exit-code` drift check
   (lines 108-112), which reads the worktree and needs no token at all. No step
   pushes, comments, uploads a release, or touches Pages: the Pages deploy
   (actions/deploy-pages, `pages: write`, `id-token: write`) lives only in
   deploy.yml:39-42, which this change does not touch. No privilege regression:
   `permissions:` can only narrow the default token, and the two new run steps
   contain no `${{ }}` expressions, so there is no script-injection path. The
   triggers remain `push` and `pull_request` (never `pull_request_target`), so
   fork PRs get the read-only token with no secrets. This closes the full
   security review's Medium finding at ci.yml:13-16. The pre-existing Low about
   tag-pinned actions (full security review, `.github/workflows/*.yml`) is
   unchanged and not repeated here.

2. PASS: NaN semantics of the six guard rewrites. All six sites
   (core/src/battery.rs:88 capacity_mah, core/src/battery.rs:91
   hover_thrust_n, core/src/motors.rs:129 time_const,
   core/src/scenario.rs:144 duration_s, core/src/sim.rs:124 takeoff_altitude,
   core/src/state_machine.rs:206 takeoff_altitude) read `x <= 0.0 ||
   x.is_nan()` inside a reject guard (`return Err(...)`), and each rejects NaN:
   `NaN <= 0.0` is false but `is_nan()` is true, so the disjunction fires and
   the error is returned, identical to the old `!(NaN > 0.0)`. Full truth-table
   equivalence with the old form: NaN rejected on both; zero (both signs) and
   all negatives and `-inf` rejected on both (`-inf <= 0.0` is true);
   positives accepted on both; `+inf` accepted on both (see the Low above:
   unchanged pre-existing behavior, not an admission introduced here). No guard
   admits NaN, `+inf`, or `-inf` where it previously did not. Each site carries
   a comment stating the NaN intent, and the sibling guards rewritten in this
   change set (sim.rs:968-1028 `collapsible_if` merges) keep the state-mutating
   `state_machine.request(...)` as the rightmost `&&` operand at all eight
   sites (sim.rs:968-975, 976-982, 983-1001, 1002-1007, 1008-1014, 1015-1028),
   so no transition event can fire or be skipped relative to the nested form.
   The untouched `duration_s <= 0.0` guards at sim.rs:650 and sim.rs:689 still
   admit NaN; that pre-existing gap is already recorded as Low in the Phase 2
   review and is not repeated as a new finding.

3. PASS: bigint to Number narrowing (src/live/sim.ts:176-183). `gpsCounts()`
   uses `Number(this.session.gps_accepted)` and siblings, typed
   `[number, number, number]`; there is no `as`, `any`, `@ts-ignore`, or
   `unknown` escape anywhere in the narrowing (grep over src/live confirms zero
   `any`/`as any`/`as unknown`/ts-ignore matches). Precision: `Number(bigint)`
   is exact up to 2^53, and the GPS counters are u64 fix counts that grow at
   GPS rate (10 Hz to 100 Hz); reaching 2^53 would take roughly 2.8 million
   years of continuous stepping, so lossy rounding is unreachable even before
   the browser DoS caps. No wrapping: `Number(bigint)` rounds, it never
   truncates or wraps, and these values are display-only (src/live/readout.ts,
   the summary at sim.ts:214-215, which arrives as plain JSON numbers via
   `summary_json`, not bigints). No allocation size, array index, or slice
   bound derives from a GPS counter, so even a hypothetical rounding error is
   cosmetic. The generated d.ts confirms the boundary types are the ones being
   narrowed (public/wasm-rust/talon_core.d.ts:114-122 declare the three
   counters as `bigint`, while sample_count/steps_done/total_steps are
   `number`), and the mirror (src/live/engine.ts:43-48) now agrees. The test
   fake returns bigint getters (src/live/sim.test.ts:188-194), so the seam is
   exercised end to end.

4. PASS: engine.contract.test.ts disk read (src/live/engine.contract.test.ts:
   16-20). The test reads exactly two path-constant, committed files with
   `readFileSync('public/wasm-rust/talon_core.d.ts')` and
   `readFileSync('src/live/engine.ts')`; there is no user, environment, or
   fixture input reaching the path (no `process.env`, no argv, no glob), so no
   path traversal is constructible. It is strictly read-only (no write, no
   import of the read content as code, no `eval`/`new Function`; the parsing is
   `indexOf` plus constant regexes, and `run_scenario\([^)]*\)` and friends are
   fixed patterns with no interpolation and no ReDoS shape). Both filenames are
   ordinary tracked files under the repository, so no symlink or untrusted-file
   concern beyond the standard "committed files are trusted" posture. The one
   environmental assumption is the relative-path cwd noted in the Low table
   entry; failure mode is a loud ENOENT test failure, never a silent pass. In
   CI the `web` job runs `npm test` from the checkout root (vitest root and
   `environment: 'node'`, vitest.config.ts:11-12), where both reads resolve.

5. PASS: no new secret, network, or dependency surface. The two new CI steps
   (`cargo fmt --check`, `cargo clippy`) are offline passes after the existing
   toolchain install; `components: rustfmt, clippy` (ci.yml:46) adds two
   standard components from the official Rust distribution to the existing
   `dtolnay/rust-toolchain` install and nothing else. No `uses:` was added or
   changed (the nine action references in ci.yml are the pre-existing set). No
   manifest change is in scope: the contract test imports only the `node:fs`
   builtin and the existing `vitest` devDependency, and no new fetch, socket,
   or process spawn appears anywhere in the change (grep over src/ and the
   three Rust crates shows only the pre-existing same-origin asset fetches and
   type names like `ControlCommand`). No credentials, tokens, or `.env` content
   in any touched file. The full security review's OSV advisory sweep of locked
   dependencies stands unchanged.

Recurring pattern reinforced: a validation guard rewritten for clippy
`neg_cmp_op_on_partial_ord` must keep NaN rejection explicitly (`x <= 0.0 ||
x.is_nan()`) and should grow `is_finite()` when the hardening pass lands; the
security-relevant invariant is the truth table, not the shape of the
expression. A test that reads generated artifacts from disk should use
path-constant, repo-relative reads and fail closed, as this one does.
