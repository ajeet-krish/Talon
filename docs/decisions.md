# Talon Decisions

Last updated: 2026-10-18

## FlightLab visual tool (2026-10-16)

### D1: Extend talon-replay in place, no new app

**Decision.** The interactive visual tool evolved the existing replay app
(then `talon-replay/`, since D16 the repository root) into a unified
present-and-run experience. No separate web app.

**Rationale.** talon-replay already owns schema consumption, playback
math, the phase timeline signature, vitest coverage, and the GitHub
Pages deploy workflow. A new app would re-buy all of that and fork the
schema contract.

**Trade-off.** The R3F component tree grows (ObstacleField, bloom
postprocessing, labels). Acceptable: the existing scene is already
componentized and the new layers are additive.

### D2: "Run" compute path is in-browser WASM (committed Phase 4)

**Decision.** Phase 4 compiles the C++17 header-only kernel with
Emscripten and runs the autopilot live in the browser. The Python glue
(NLGL, mission, state machine, battery, motors) is ported to TypeScript
or trimmed for the sandbox.

**Rationale.** The user selected WASM over a local Python backend or
precomputed sweeps. WASM is the only path that works on static GitHub
Hosts for every visitor with zero server. The zenith-web project already
proves the Emscripten + golden-master parity pattern in this portfolio.

**Trade-off.** Highest effort and parity risk. Mitigated by staging:
Phase 1 proves the obstacle narrative offline; Phase 4 ports the kernel
with golden-master fixtures against the Python SIL.

### D3: Obstacles are real sim data, not decoration

**Decision.** Phase 1 adds a deterministic leg-detour planner
(`talon.avoidance`) that inserts detour waypoints into the mission.
The full EKF-in-the-loop SIL runs the augmented mission and CI asserts
minimum 3-D surface clearance > 0.

**Rationale.** The user chose "real avoidance, staged." A detour planner
is pure Python, deterministic, unit-testable, and requires no change to
`guidance.py` or the C++ kernel. It turns the obstacle field from a
visual sticker into an engineering substance differentiator versus
SwarmGNC's cosmetic debris.

**Trade-off.** At most one detour per leg (v1). Multiple obstacles on
one leg need a richer planner (v2, reactive potential field). Acceptable
for the Phase 1 circuit.

*Superseded in part.* The geometry and the planner are no longer pure Python:
they live in `core/src/avoidance.rs` and the Python module re-exports them
(D35), and the autopilot flies a reactive law that takes several zones on one
leg one after another (D33, D34). The v2 "reactive potential field" named here
became a latched bypass point instead, because a latched bypass can be released
on a stated clearance condition and a potential field cannot be reasoned about
the same way. The offline planner and its one-detour-per-leg limit remain in
use for the recorded obstacle-field scenario.

### D4: Schema 1.1 with optional obstacles, backward compatible

**Decision.** `SCHEMA_VERSION = "1.1"`. The `obstacles` top-level key
is written on every export. Old 1.0 documents (without the key) remain
valid; the validator checks the block only when present.

**Rationale.** The exporter is the single source of truth for the JSON
contract. Keeping the validator lenient on the new key means existing
`plots/json/*.json` files and the web app TS types do not break on
the first deploy.

### D5: Matplotlib video pipeline untouched

**Decision.** `animation.py`, `plotting.py`, and the four docs/videos
MP4s are preserved exactly. The obstacle scenario adds a fifth video
(`obstacle_field.mp4`) via the same `export_flight_video` path.

**Rationale.** The user explicitly likes the professional matplotlib
waypoint videos. They are portfolio deliverables. The web tool is the
interactive sibling, not a replacement.

### D6: SILConfig and SILResult carry obstacles

**Decision.** `SILConfig.obstacles: list[SphericalObstacle]` and
`SILResult.obstacles: list[SphericalObstacle]` are new fields with
empty-list defaults. The exporter serializes `SILResult.obstacles` to
the JSON document.

**Rationale.** The scenario defines the obstacle field, passes it
through SILConfig to the runner, and the runner copies it into the
result. This keeps the obstacle metadata with the flight data end to
end.

## Interactive obstacle scene (2026-10-17)

### D7: Restrained bloom, not a full SwarmGNC haze

**Decision.** Add `@react-three/postprocessing` bloom with
`intensity 0.55`, `luminanceThreshold 0.6`, `mipmapBlur`, so only the bright
emissive accents glow (drone nose, waypoint tags, keep-out shells). The scene
keeps its crisp instrument look; the glow reads as energy, not atmosphere.

**Rationale.** The user asked for the SwarmGNC aesthetic, but Talon's visual
identity is a disciplined instrument panel, not a cinematic space scene. A
high-luminance-threshold bloom takes the one idea that carries across (glowing
hazard volumes and markers) without repainting the whole app.

**Trade-off.** Less dramatic than SwarmGNC's heavy bloom. Acceptable: the
obstacle field already carries the visual drama.

### D8: Clearance is data-driven, not decorative

**Decision.** `ObstacleField` tints each keep-out zone by the truth
trajectory's live surface clearance: amber clear, yellow caution (under the
2.0 m planner margin), rose breach. The HUD prints the live clearance and
names the nearest zone; the legend prints the flight minimum.

**Rationale.** Matches the project ethos already set by `WindParticles`
("data-driven ambient motion, not decoration") and the covariance ellipsoid
("the estimator's honesty meter"). The keep-out volume becomes a live sensor
readout, not a static sticker, and reuses the exact quantity CI asserts.

**Trade-off.** Requires per-frame clearance math in the render loop (cheap:
three obstacles, one hypot each).

### D9: Canvas sprite labels, not drei Html / CSS2D

**Decision.** Keep-out labels are canvas-rendered sprite textures
(`makeObstacleLabel`), matching the existing `WaypointMarkers` pattern,
rather than drei `Html` or a CSS2DRenderer overlay.

**Rationale.** Sprites are offline-safe, allocation-light, unaffected by DOM
layout, and consistent with the one label pattern already in the codebase.
SwarmGNC's CSS2D labels would introduce a second label technology for no
visual gain at this scale.

### D10: One clearance module shared by scene, HUD, and legend

**Decision.** All clearance math and the keep-out palette live in
`src/utils/clearance.ts`. The scene tint, the HUD readout, and the legend
import from it; none define their own.

**Rationale.** Three surfaces read the same quantity; a single source
prevents the readout and the scene from disagreeing. The module is pure, so
it is unit-tested in the node vitest environment without a DOM or WebGL.

### D11: Four-section shell on the existing store clock

**Decision.** The tool is one entry point with four sections (Replay,
Telemetry, Metrics, Videos), switched by `activeView` in the existing
Zustand store. No router and no URL state.

**Rationale.** The sections share one active scenario and one playback
clock; a router would add history handling for a single-page instrument with
no deep-link requirement. Section state is ephemeral tool state, which is
what the store already holds (scenario, time, toggles).

**Trade-off.** Sections are not shareable by URL. Acceptable for a
GitHub-Pages instrument; a router can be layered on later without moving the
store.

### D12: Charts and run summary derived client-side, gates from the driver

**Decision.** The telemetry series and the metrics run summary are computed
in the browser from the recorded samples (`deriveTelemetry`, `summarizeRun`
in `src/utils/telemetry.ts`). The validation gates table is fetched from
`plots/json/validation.json`, a machine-readable sidecar written by
`sims/run_all.py` and copied into `public/data/` by `scripts/copy_data.sh`.

**Rationale.** Derived metrics cannot drift from the charts because both
read the same payload; the gates cannot drift from the driver because they
are the driver's own rows. Hand-transcribing either into a TypeScript
constant would create two sources of truth.

**Trade-off.** The driver grows a small JSON writer and the gates refresh
only when `run_all.py` and `copy_data.sh` are re-run. Acceptable: the report
was already regenerated on every driver run.

### D13: Hand-rolled SVG charts, no charting dependency

**Decision.** The telemetry charts are a small SVG component (`LineChart`)
built on pure helpers in `src/utils/chart.ts` (min/max bucket decimation,
padded domains, scales, path builder). No charting library is added.

**Rationale.** The tool needs seven single-series time plots sharing one
cursor, not an interactive charting framework. The project already
hand-rolls its canvas labels and scrubber and keeps its dependency list
deliberately short. The helpers are pure, so they are unit-tested in the
node vitest environment without a DOM or WebGL.

**Trade-off.** No zoom, brush, or tooltips out of the box. Acceptable for an
instrument readout, and the min/max decimation preserves spikes.

### D14: Only the cursor subscribes to the playback clock

**Decision.** Each chart's static geometry (decimated path, domain, grid) is
memoized on the payload; only the playhead cursor and the live value readout
subscribe to `currentTime`.

**Rationale.** The store ticks about sixty times a second during playback.
Re-deriving a roughly seven-hundred-point path per chart per frame would
waste work; a memoized path plus two small subscribers keeps a playing
scenario cheap.

**Trade-off.** One extra component boundary per chart. Acceptable and local.

### D15: Recordings staged into public/videos by copy_data.sh

**Decision.** The five MP4s are copied from `docs/videos/` into
`public/videos/` and committed, mirroring how `plots/json/*.json`
is copied into `public/data/`.

**Rationale.** The recordings are small (about 1.6 MB total) and static
hosting must serve them from `dist`. Committing keeps the app self-contained,
so a build never depends on the archive layout or a prebuild copy step.

**Trade-off.** The MP4 bytes exist twice in the repository. Acceptable at
this size; regenerate with `copy_data.sh` after `run_all.py`.

## Repository restructure (2026-10-17)

### D16: Web app at the repository root, core/ and python/ for the rest

**Decision.** The web app moved to the repository root (it is the product:
Talon FlightLab). The header-only C++ kernel moved to `core/`, and the Python
package, simulations, and tests moved to `python/`. Vite `base` becomes `/`.

**Superseded in part by D21.** The base is now `/Talon/`: the repository is a
GitHub Pages project site served from a subpath, so a root base emitted asset
URLs one level above the site.

**Rationale.** One repository, one product: the simulator ships as the root
app rather than as a `talon-replay/` subfolder. The three trees gain single
owners (web, C++, Python), which also removes the `src/` collision between the
web source and the previous Python `src/` layout.

**Trade-off.** The Python package moved out of the idiomatic `src/` layout, so
`pyproject.toml`, the driver's root math (`PY_ROOT`/`REPO_ROOT`), CMake's
extension output directory, CI, and the docs had to move with it. Mitigated by
re-running every suite at the new paths (see docs/tests.md).

**Supersedes** the location named in D1, not its intent (still one app, no
fork).

## In-browser live run (Phase 4, 2026-10-17)

### D17: The estimator and plant stay in C++; only the glue is ported to TypeScript

**Decision.** The WebAssembly module (`wasm/exports.cpp`, built by
`wasm/build.sh` with Emscripten + embind) owns dynamics (RK4), Dryden wind,
the cascaded PD controller, and the 15-state MEKF. The Python glue that has
no native counterpart is ported to TypeScript under `src/live/`: mission
bookkeeping, NLGL guidance, the motor mixer and first-order lag, the
truth-side IMU/GPS sensor models, the flight-phase state machine, and a
line-for-line mirror of the SIL orchestration loop (`sim.ts`).

**Rationale.** The estimator and controller are the safety-critical math; a
second implementation would be a second source of truth and would silently
diverge from the validated Python/C++ stack. The glue is thin, deterministic,
and already exercised by the Python suites, so porting it is cheap and its
behavior is checkable against Python-computed fixtures.

**Trade-off.** The browser and the recorder share the kernel but not the
sensor-noise realization (see D18), and the TypeScript port must be kept in
step with the Python glue by hand. Parity is enforced by fixtures, not by
shared code.

### D18: Statistical, not bit-exact, noise parity; fixtures for the deterministic math

**Decision.** The TypeScript noise generator is mulberry32 seeded per run with
Box-Muller normals, not numpy's PCG64/ziggurat. Browser runs are therefore
statistically equivalent to the recorded Python runs but not bit-identical.
Parity is enforced on the deterministic math instead:
`python/sims/export_live_configs.py` computes reference outputs from the
Python modules (NLGL cases, a waypoint sequence, the mixer and lag, plus
WASM-side propagate/EKF/controller values) into
`fixtures/live_fixtures.json`, and the vitest suites assert the TS and WASM
sides match within tolerance. `fixtures/sil_reference.json` (optional, from
`--with-sil`) carries two full Python SIL trajectories for a statistical
end-to-end check.

**Rationale.** Matching numpy's exact normal stream would mean porting
PCG64 plus the ziggurat tables, a large surface with no engineering payoff:
the noise is already a modeling choice, and every acceptance number is a
statistic. The deterministic math is where a port silently breaks, so that is
where parity is pinned.

**Trade-off.** A browser run cannot be replayed byte-for-byte against the
recorded scenario JSON. Documented in the Run view and here; the integration
test compares against the Python SIL trajectory within a stated tolerance.

### D19: A missing WASM module disables Run rather than falling back to a TS MEKF

**Decision.** If the Emscripten module fails to load (not built, blocked, or
unsupported), the Run section shows an explicit "kernel not loaded" state and
disables the run button. Every other section is unaffected. There is no
TypeScript MEKF fallback.

**Rationale.** A JS estimator would (a) be a second source of truth for the
hardest math and (b) produce results that look authoritative while differing
from the validated filter, which is worse than an honest "unavailable".

**Trade-off.** A checkout without Emscripten cannot demo the live run. The
committed `public/wasm/` artifacts mean the deployed site always can.

### D20: The live run covers the nominal mission; fault paths stay recorded-only

**Decision.** The four runnable live configs are hover, waypoints, obstacle
field, and wind. GPS outage, EKF divergence/emergency, and low-battery RTL
are not injectable in the browser, the battery state of charge is held at
1.0 so the low-battery guard never fires, and the obstacle-field waypoints
are passed pre-planned (the Python detour planner runs offline, in the
exporter, not in the browser).

**Rationale.** The three fault scenarios are about FDIR behavior and are
already recorded and inspectable in Replay; re-running them live would add a
large amount of ported fault-injection surface for no new insight. Porting
`avoidance.py` would duplicate a planner that is not needed for a
pre-planned mission, and porting `battery.py` would add a fifth state model
only to keep the low-battery guard dormant.

**Trade-off.** The live run cannot demonstrate FDIR. Stated in the Run view
copy and in docs/tests.md; the recorded scenarios keep that coverage.

## Pages deployment (2026-10-18)

### D21: Project-site base path and GitHub Actions as the Pages source

**Decision.** Vite `base` is `/Talon/`, defaulted in `vite.config.ts` and
overridable with `VITE_BASE`. The deploy workflow derives the same value from
`GITHUB_REPOSITORY` (project pages under `/<repo>/`, a `<name>.github.io` page
at `/`) and passes it as `VITE_BASE`, asserts that the built HTML carries the
prefix, and adds `wasm/**` to its trigger paths. The repository's Pages source
is **GitHub Actions**, and `public/.nojekyll` is committed.

**Rationale.** The repository is `ajeet-krish/Talon`, so Pages serves the site
from `https://ajeet-krish.github.io/Talon/`. With `base: '/'`, `dist/index.html`
referenced `/assets/index-*.js`, which resolves one level above the site and
404s, leaving a blank page. That fault was invisible because the repository's
Pages source was still `legacy` with source `main` `/`: GitHub published the
raw repository through Jekyll, so `/Talon/` returned the untranspiled source
`index.html` (`script src="/src/main.tsx"`) while the Actions workflow failed at
`actions/configure-pages@v5` with "Get Pages site failed ... configured to build
using GitHub Actions". Deriving the base from the repository name in CI keeps it
correct across a rename, and the guard step turns a base regression into a build
failure instead of a silently blank deployment.

**Trade-off.** A root-served local session now needs `VITE_BASE=/ npm run dev`,
and the dev and preview servers serve under `/Talon/` rather than `/`. That is a
visible change, but it reproduces production paths exactly, which is what would
have caught this class of bug. The workflow's `enablement: true` lets
`configure-pages` switch the source to Actions by itself when the token has the
`pages: write` scope; otherwise the setting must be flipped by hand.

## Rust core rewrite (2026-10-18)

### D22: The autopilot, guidance and estimation core is Rust, in one Cargo workspace

**Context.** The GNC core was C++ header-only (`core/include/talon/*.hpp`)
exposed to Python through pybind11 and to the browser through Emscripten. Every
behaviour change meant touching three implementations: the C++ kernel, the
TypeScript glue under `src/live/`, and the Python glue under `python/talon/`.
The live run, the primary deliverable, lived in the TypeScript copies, so the
validated C++ kernel was not the code the browser actually ran.

**Decision.** Rebuild the core as Rust, and make it the only implementation of
dynamics, wind, control, estimation, guidance, mission logic, the flight-phase
state machine, sensors, motors, battery and the SIL runner. One Cargo workspace
at the repository root: `core/` is the `talon-core` library, `bindings/` is the
PyO3 crate that exposes it to Python as `talon._core`, `wasm/` is the
wasm-bindgen crate for the browser. The C++ core, CMake, pybind11, Emscripten,
GTest and the `src/live/` TypeScript glue are deleted once the port is proven
against the Python modules as a parity oracle. `nalgebra` fixed-size matrices
replace Eigen, so a dimension mistake is a compile error.

**Rationale.** One implementation means one place for a behaviour to be wrong and
one place to test it. Rust also gives the same code path on native and wasm32
without a second language for the browser, and PyO3 keeps the Python analysis and
plotting harness, which is the part worth keeping. Fixed-size nalgebra matrices
turn a whole class of latent shape bugs into compile errors. The measured
motivation for a native loop was not raw speed: flops were never the bottleneck.
It was boundary chattiness, about 21 scalar reads per control tick through the
JS to WASM edge in `recordHistory` alone. A native-owned loop with zero-copy
typed-array views reduces that to about one call per frame and unlocks runs far
faster than real time.

**Trade-off.** The rewrite is large and, until it lands, the repository carries
two cores. Recorded telemetry, the validation report and the five videos are
frozen as produced by the Python stack, so the live runs are statistically
equivalent to them and not bit-identical. That is acceptable because the visible
difference is estimator noise, not dynamics.

### D23: libm for transcendentals, and the Rust types are the schema

**Context.** Two smaller decisions inside D22, each with a concrete failure mode
if taken the other way.

**Decision.** Every transcendental in `core/src/math.rs` routes through the
`libm` crate rather than `std`, so native and wasm32 execute the identical
algorithm and produce bit-identical results across targets. The flight document
types in `core/src/types.rs` are the schema source of truth, serialized by serde
with unchanged `snake_case` field names, and the TypeScript and Python mirrors
become generated artifacts (`ts-rs` and the PyO3 structs) rather than
hand-maintained copies.

**Rationale.** `std`'s float functions are not specified by the Rust standard to
be the same across platforms, and the whole point of the browser run is that it
reproduces the native run. Pinning the algorithm retires the D18 "statistical
only" caveat for cross-target reproducibility. On the schema: 287 lines of
hand-written type mirrors exist today across `src/types/flight.ts`,
`src/live/types.ts` and `python/talon/types.py`, and every one of them is a
place where a field name can silently drift from what the exporter writes.

**Trade-off.** `libm` is a little slower than the platform `libm`, which does not
matter because the bottleneck was never arithmetic. Generating the mirrors adds a
build step and means the mirrors must not be edited by hand, which is the point.

### D24: The browser steps the native core through a wasm-bindgen session

**Context.** The Rust core is the autopilot (D22). The browser has to run it, and
the first thing that needs to exist is a page where the browser actually does.

**Decision.** `wasm/` is a thin wasm-bindgen crate exposing one `Session` object:
construct it from scenario JSON, `prepare` once, `step` or `advance(n)` inside a
frame budget, read the live pose out at any point, and pull the finished flight
document and run summary at the end. The plant loop, guidance, mission manager,
motor mixer, sensor suite, phase machine, and FDIR all live in the native core
and are reached through that one boundary. The TypeScript side
(`src/live/engine.ts` plus a small `LiveSim`) only loads the module and drives
it; it owns no autopilot logic.

**Rationale.** The measured motivation (D22) was boundary chattiness, and a
stepwise session is what removes it: the frame loop makes about one call per
frame rather than a scalar read per control tick, and reading the live pose is
three calls when the view needs it. A stepping API also serves both consumers
without a second code path, since a batch run is just `run_to_end`. wasm-pack's
`--target web` output is loaded at runtime from the Vite base URL rather than
bundled, so the module stays out of the JS graph and a missing build degrades to
the existing "Run disabled" message (D19) instead of a failed bundle.

**Trade-off.** No node test can load the module the way the browser does
(`fetch` cannot read a `file:` URL), so the native session is covered by Rust
tests and the browser path by a build-and-serve check; the TypeScript wrapper is
covered against a scripted fake session. Real-time pacing is deliberately not
part of this step: the loop runs as fast as the frame budget allows, which is
what the Phase 4 loop already did, and turning that into a real-time GCS
playback is Phase 6.

### D25: The flight document is assembled in Rust, and the schema does not move

**Context.** The recorded scenarios, the scene, the charts, the metrics, and the
run summary all speak schema 1.1 (D4). A live run has to produce something they
already understand, or the whole presentation layer forks.

**Decision.** `talon-core::export` assembles the schema 1.1 document
(`core/src/export.rs`), preserving the two exporter details that matter: floats
round to six decimals, and `meta.duration_s` is the last sample time rather than
the requested duration. Field names, units, and ordering are unchanged, and the
document is serialized with serde from the Rust types that are the schema source
of truth (D23). The one new input is `meta.generated_utc`, supplied by the caller
because the core has no clock.

**Rationale.** A live run and a recorded run are then the same document and every
downstream consumer stays untouched, which is what makes the completion of a
browser run a rail entry next to the recorded scenarios. Keeping the exporter in
the core rather than in the wasm crate means it is covered by ordinary
`cargo test` on native, where a schema drift is a failing assertion.

**Trade-off.** The document is a whole-history JSON, so a 120 s run is a few
megabytes of text parsed on completion rather than a frame stream. That is fine
for the current UI, which shows a finished run, and the zero-copy typed-array
views the live GCS display wants are Phase 6 work on the same session object.

### D26: The live run is paced on the page, and the GCS has its own view

**Decision.** Phase 6 adds a dedicated live view: its own scene, its own
instrument rail, and a playback rate control (1x, 4x, 16x, and a maximum
limited only by the frame budget). Pacing lives entirely in TypeScript
(`src/live/pacing.ts`): the run view asks how far the run should have got by
now and advances the native session by that many plant steps, then reveals
whatever it recorded. The core steps as fast as it is told and has no notion
of time passing. On completion the run publishes its document through
`setLiveRun` and the replay, telemetry, and metrics sections work unchanged.

**Rationale.** The alternative considered was to grow a `FlightData` document
in the store as the run progressed, which would have reused the existing
scene. It was rejected for three concrete reasons. `src/utils/telemetry.ts`
takes `data.time_s.length` as N, so the metrics view would report a different
flight depending on when it was opened. `TrajectoryRibbon` builds its geometry
once per document and would need a refill hook plus a camera framed over a
trajectory of preallocated zeros. And every frame would reconcile a document
whose arrays are hundreds of kilobytes. A separate view keeps the recorded
path exactly as it was and gives the live path a recorder sized to the
scenario, filled from a streamed delta.

Pacing on the page rather than in the core is what keeps a watched run and a
batch run the same flight: the core's trajectory does not depend on how it was
revealed, so the numbers a viewer reads are the numbers a batch run produces.
`Infinity` is offered as a rate because the fastest a run can go is a
legitimate thing to want, and it costs nothing to express once pacing is a
display concern.

**Trade-off.** The web app now assembles a scene twice: the recorded scene
(`src/components/scene/`) and the live one (`src/components/live/`). They share
the ground plane, the phase palette, and the clearance helpers, which is what
keeps them looking like one tool, but a change to the scene language has to be
made in both places. The rail can not reuse `src/components/charts/LineChart.tsx`
either, because its cursor and live value come from the store clock; it has its
own strip chart over the recorded series.

### D27: A sample's phase is tagged in the recorder, and altitude is the up axis

**Decision.** Two consequences of showing a live run. The recorder tags every
sample with the flight phase in effect when it was recorded, as an index into a
fixed palette order, so the live trail can colour itself from the data the
recorder already has (the core's phase timeline arrives as events, not per
sample). And `nedToScene` is `[north, -down, east]`: NED index 0 is north, index
1 is east, index 2 is down, and the scene puts down on -y so up is up.

**Rationale.** The phase tag keeps presentation out of the recorder (it stores
an index, not a colour) while avoiding a scan of the timeline per vertex.

The axis mapping corrects a real bug that this phase surfaced. The scene had
been reading index 1 as up and index 2 as depth, contradicting its own comment
and every other consumer: `deriveTelemetry` reads altitude as the negated index
2, `python/talon/plotting.py` plots `home_d - truth_position[:, 2]` for altitude
and `truth_position[:, 1]` for east, and the recorded climbs show about
-2.5 m/s on index 2 during takeoff. So the charts said the vehicle climbed to
10 m while the scene drew it at whatever its east coordinate happened to be,
mostly behind the opaque ground disc. The heading had the same misreading
(`atan2(north, down)`), as did the wind field's advection.

**Trade-off.** None worth naming: the mapping is now a documented rotation with
a determinant of +1, so geometry is not mirrored, and the replay framing and
the vehicle's nose both changed for the better. The change touches the replay
view as well as the live one, which is correct because it was wrong in both.

### D28: The Python extension is PyO3, and the retired stacks are gone

**Decision.** `bindings/` is a PyO3 crate (`talon-py`) that publishes the
`talon._core` extension module over `talon-core`. With that in place the C++
stack was deleted outright: `core/include/talon/`, `core/tests/`, the root
`CMakeLists.txt`, `bindings/pytalon.cpp`, `wasm/CMakeLists.txt`,
`wasm/src/exports.cpp` and the Emscripten output `public/wasm/`. The
`src/live/` TypeScript autopilot modules went in the same pass: `guidance.ts`,
`mission.ts`, `motors.ts`, `sensors.ts`, `stateMachine.ts` and their tests,
plus `src/live/math.ts`, whose only remaining role was a duplicated `Vec3`.

**Rationale.** One implementation of the flight-critical math, reached two
ways: `wasm/` for the browser and `bindings/` for Python. Keeping a second
implementation alive would be a second source of truth for the parts a
reviewer is most likely to check.

**Evidence.** The 186 pytest tests were written against the C++ kernel and
were not edited while the kernel changed. They pass against the Rust `_core`
unchanged, so the port is behaviourally pinned by the pre-existing suite
rather than by a new oracle. The Python tests that the deleted TS modules
mirrored are covered in Rust by 16 `guidance`, 13 `mission`, 15 `motors`, 13
`sensors` and 21 `state_machine` tests.

**Trade-off.** The Python column of the module map stops being the reference
implementation. A Python SIL run is now hybrid by construction: plant,
controller, estimator and wind come from the Rust `_core`, while guidance,
mission, motors, sensors, the phase machine and the MAVLink edge stay Python.
That is deliberate, because the recorded telemetry, the validation report and
the five videos are frozen artifacts produced by that harness and are not
regenerated; rewriting the harness would invalidate them for no gain.

### D29: The PyO3 boundary is Python sequences, and the hot paths get their own accessors

**Decision.** Vector fields cross the boundary as `list[float]`; setters use
`try_iter` so a numpy array, a list, a tuple or a generator all work.
`Ekf.position_variance()` and `Ekf.position_covariance_trace()` return the
three position diagonal entries and their sum, so the 200 Hz convergence check
never builds a 15x15 matrix. `covariance()` still returns the full list of
rows for the tests. `Ekf.nominal()` returns a snapshot and a new
`Ekf.set_nominal(state)` writes one back; `mutable_nominal()` and
`mutable_covariance()` are not exposed.

**Rationale.** The pybind11 module exposed `Eigen` types through rust-numpy's
equivalent; doing that here would add a second dependency and a second ABI to
keep aligned for a boundary that moves three or six numbers at a time. A list
of floats is `np.asarray`-able for free and is trivially inspectable. Measured
cost is nil: the pytest suite runs in 8.6 s against the Rust extension, the
same as it did against C++. `mutable_nominal()` cannot be expressed safely
under PyO3's ownership model (it returned a live reference into the filter),
and it had exactly one caller, which now reads a snapshot and writes it back.

**Trade-off.** `covariance()` copies 225 floats per call, and only tests call
it. The Python harness's list-typed fields are copies, not views, so in-place
element assignment (`state.position[0] = x`) is not supported; nothing in the
tree did that, and the harness assigns whole fields.

### D30: The extension build resolves its own interpreter

**Decision.** `bindings/build.sh` builds with `--features extension-module` and
copies the `cdylib` to `python/talon/_core<EXT_SUFFIX>`. `.cargo/config.toml`
sets `PYO3_PYTHON` to `.venv/bin/python` (overridable), and
`bindings/build.rs` calls `pyo3_build_config::add_extension_module_link_args()`
so the macOS `cdylib` resolves CPython symbols at load time. The
`extension-module` feature is not a default, so `cargo test -p talon-py` links
libpython and runs the marshalling tests natively.

**Rationale.** Without the venv pointer, cargo and pytest can disagree about
the interpreter, and on macOS the pybind11-era CMake find-module behaviour is
not available to reproduce the old setup. PyO3 delegates the extension-module
link flags to the consuming build script, which is easy to miss and produces a
confusing `_PyObject_*` link failure; doing it in `build.rs` fixes it once.

**Trade-off.** `.venv` must exist before `cargo test --workspace` can link the
binding tests, which is already the documented order (`uv sync` first). The
x86_64 and Linux equivalents of the link flags are handled by the same call.

## Altitude legibility (Phase 7, 2026-10-18)

### D31: Altitude is stated by a mission-derived ladder and a screen-space plumb line

**Decision.** The live scene gains a graduated altitude ladder standing at the
pad (`components/live/LiveAltitude.tsx`), and the vehicle's plumb line is drawn
at a constant on-screen width. The ladder's rungs come from
`altitudeLadder(cfg)` in `src/live/view.ts`, which is pure and unit tested: a
measurement grid sized from the mission's own scale (a `niceStep` interval from
0.5 m to 100 m, at most six rungs), with the rung at an altitude the mission
actually flies drawn longer and in the signal colour, and the mission's ceiling
always present as a rung. Flown altitudes are the takeoff column and every
waypoint's altitude; ground level is excluded, it is the datum. The ladder is
static, so it costs no per-frame work.

The plumb line keeps its constant model radius of 1.2 cm but is scaled per
frame by the camera distance (0.0016 world metres of radius per metre of
distance), so it stays a hairline at any zoom, and its opacity grows a little
with height so a long line carries a little more ink.

The replay scene is deliberately untouched: it keeps its shadow cue and its
ground-track yaw.

**Rationale.** Altitude is the one quantity a perspective camera cannot read.
The shadow says where the vehicle is over the ground and the plumb line says
how far above it, but neither states a number: an absolute claim needs
something that stands still to measure against. The rungs must come from the
mission rather than a generic 5/10/20 ladder, because a ruler with the wrong
major marks is worse than no ruler, and the run's own flight level is the
interesting mark.

The fixed-radius stem was measurably broken, which is what surfaced the phase.
At the default circuit frame the camera sits about 79 m from the pad, the
viewport covers roughly 60 m vertically, and a 1.2 cm cylinder is therefore
about 0.14 px wide: the cue was absent exactly when the scene was most
cluttered.

**Trade-off.** A world-scaled sprite label cannot be readable across the whole
mission scale, and this scene labels with sprites rather than DOM overlays
(D-labels decision in `labels.ts`). At the default frame for the two circuit
scenarios the rungs read but the numbers need the pad zoom an operator would
use to read a ruler anyway; for the hover and wind scenarios, whose missions
are small, the numbers are readable immediately. The alternative was to keep
the numbers readable at 80 m by making the quads about 4 m tall, which would
have dwarfed the vehicle. The second trade-off is asymmetry: the replay views
now state altitude less loudly than the live view does, because this phase was
scoped to the live ground station.

### D32: Scene code that cannot be rendered in CI is guarded by pure tests and the build

**Decision.** The ladder's decision logic lives in `src/live/view.ts` as pure
functions (`niceStep`, `missionAltitudes`, `altitudeLadder`) with vitest
coverage, including a test that reads the committed
`public/data/live/configs.json` and asserts every served scenario has a ladder
whose ceiling is its highest flown altitude. The React components hold only
geometry and materials, and are guarded by the typecheck in `npm run build`.

**Rationale.** The repository has no browser in CI: `npm run smoke:wasm` flies
the wasm engine headlessly but never touches the R3F tree, so a scene component
can only be verified by its build. Pushing the arithmetic into pure functions is
the only way to make the interesting part of a visual change checkable, and
reading the shipped configs rather than a fixture makes the drift check real
(the ladder is derived from the same file the browser loads at run time).

**Trade-off.** Geometry, colour, and proportions are not covered by anything
automated; they were reasoned about from the mission scale and the default
frame distance, and a human still has to look at the page. This is recorded as
a residual risk rather than papered over.

## Reactive avoidance, native (Phase 9a, 2026-10-18)

### D33: Avoidance is a core layer, and the offline planner becomes mission planning

**Decision.** `core/src/avoidance.rs` holds the avoidance arithmetic:
`leg_obstacle_distance`, `leg_intersects_obstacle`, `compute_detour_point`,
`plan_avoidance_mission`, `min_surface_clearance`, and the reactive law
`ReactiveAvoidance`. The loop consults the law during the WAYPOINT phase only,
and the directive it returns replaces the guidance target for that control
tick; the mission's waypoint bookkeeping, acceptance radius, and the phase
machine are untouched. `SimConfig` gained `obstacles` and `avoidance`, fed from
the scenario's existing `obstacles` list plus a new optional `avoidance` block,
and the law is off by default. The run result and the live summary report
`min_obstacle_clearance_m`, measured on the truth trajectory rather than on the
estimate the law steers with, and `avoidance_activations`.

**Rationale.** The standing requirement is that the autopilot, guidance, and
avoidance be primarily native. Until this phase the core carried an `Obstacle`
type that nothing in the loop read: the only avoidance in the tree was an
offline Python planner, run before the flight, whose detoured waypoints the
exporter bakes into the browser configs. The vehicle therefore flew a
pre-computed path and never avoided anything itself. Putting the law beside
guidance also means it acts on the same state estimate as the rest of the loop,
so its behaviour under estimator error and wind is the behaviour of the real
system rather than a re-simulation.

Off by default is what protects the frozen artifacts: the four shipped live
configs and every recorded run keep their paths bit for bit, which
`zones_alone_do_not_change_the_flown_path` asserts by comparing truth histories
with the zones present and absent while the law is off.

**Trade-off.** Two implementations coexist for one phase: the Python planner
still owns the offline detour that the recorded obstacle-field run used. The
next sub-phase collapses them by making the Python module delegate to `_core`.
The law is also deliberately narrower than a general planner: one latched
bypass at a time, horizontal decisions at the leg's altitude, no vertical
escape. Zones abreast of each other on one leg are taken sequentially rather
than with a single pre-planned detour, which is more than the offline planner's
v1 manages, but a zone wider than the detour reach, or a corridor of zones, is
not solved and is not claimed to be.

### D34: The law latches one bypass, releases on an along-track band, and never releases inside the margin

**Decision.** The law steers a threatened leg by substituting a bypass point
for the waypoint. A zone counts as a threat when its horizontal keep-out shell
(`radius + clearance_margin`) covers the leg, its abreast point lies inside the
leg, and it sits between 2 m and the lookahead ahead of the vehicle, where the
lookahead is the lookahead time times the ground speed with a 2 m floor. The
bypass is the closest-approach point stepped radially away from the zone by
`radius + margin + 0.5 m`, at the leg's altitude. The latch is given up when
the clearance has recovered to the margin and either the vehicle has drawn
level with the zone or it is within a release radius of its bypass. A new
active waypoint index retires the latch, and so does leaving the WAYPOINT
phase.

**Rationale.** The lookahead has to grow with ground speed or the law commits
too late at cruise and the deviation reads as a swerve; the floor keeps it
working at a hover. The release needed the hygiene. "Passed the zone" and
"reached the bypass" each fail alone: a vehicle flying to a bypass arrives at
the same along-track coordinate as the closest approach and cannot pass it
while it is still being steered at, while a release that only checks
"past the zone" lets the vehicle turn back into a zone it has not cleared. The
along-track band makes latching and releasing disjoint in distance, so the law
cannot chatter, and the clearance gate makes it impossible to let go while
inside the margin. Flown closed loop on a straight-through mission against a
4 m zone with a 2 m margin, the law took one bypass and held 3.24 m of
clearance while still reaching every waypoint.

**Trade-off.** The bypass stands off by a fixed buffer rather than a
time-optimal path, so the detour is slightly wider than it strictly needs to
be, and the release radius is a heuristic (half the standoff, floored at 2 m)
chosen so that a bypass guidance smooths early still clears. Both are stated in
the module rather than tuned silently, and the closed-loop tests are what keep
them honest.

## One avoidance implementation (Phase 9b, 2026-10-18)

### D35: Python delegates to the core, and the browser gets a straight-through demo

**Decision.** The PyO3 extension publishes the avoidance boundary:
`leg_obstacle_distance`, `leg_intersects_obstacle`, `compute_detour_point`,
`plan_avoidance_mission`, and `min_clearance`. `python/talon/avoidance.py`
becomes a thin re-export over `talon._core`, like `controller.py` and `ekf.py`
before it, and no longer contains geometry. The Python objects stay the types
the harness already uses: obstacles and waypoints cross as duck-typed
attributes (`center_ned`, `radius_m`; `position`, `speed`, `heading`,
`radius`), and the planner hands back tuples the wrapper wraps in
`talon.types.Waypoint`. A negative clearance margin is now rejected up front by
every entry point that takes one, rather than only where a leg and an obstacle
happen to meet the geometry.

The exporter gains a fifth live config, `avoidance`: the same circuit and the
same three keep-out spheres as `obstacle_field`, but with the plain legs and
the law switched on, so the browser shows the autopilot deviating around each
zone itself. The recorded `obstacle_field` scenario is untouched and still
flies its pre-planned detours.

**Rationale.** Two implementations of one geometry is the thing this phase
exists to remove, and the frozen recorded data is what proves it worked rather
than the assertion that it did: `test_avoidance.py` runs unchanged against the
delegated module, all 186 Python tests pass, and regenerating
`public/data/live/configs.json` leaves the four existing entries identical
field for field, including `obstacle_field`'s eight detoured waypoints. The
demo config exists because a capability nothing exercises is a claim, not a
feature: until this phase no shipped scenario had the law on, so the browser
never showed the vehicle avoiding anything by itself. Writing the margin into
that config (rather than inheriting the default that happens to match) keeps
the number the smoke check reads the achieved clearance against visible in the
artifact.

**Trade-off.** The clearance *readout* is still computed client-side, from the
streamed samples and the config's zones, because it has to update every frame
while the run is in flight; the wasm summary now reports the same quantity for
the end of a run, and the two are consistent by construction but are still two
expressions of one line of arithmetic. The avoidance *decision* has one
implementation, which is the part that matters: the geometry, the lookahead,
and the latch exist once, in Rust, and the Python planner and the browser
autopilot both run that code.

## Open decisions (resolved at Phase 4)

- Acceptance metrics for v2 reactive avoidance: resolved by D33, D34, and D35
  (the law is native and in the loop, it reports the clearance it achieved
  against the margin it was given, and the offline detour planner delegates to
  the same geometry).
- WASM boundary: resolved by D17 (kernel in WASM, glue in TypeScript).
- FlightDataSource reuse: resolved by keeping the recorded whole-history
  `FlightData` path and having the live run emit the same `FlightData`
  document on completion, rather than a parallel frame stream. The scene and
  chart layers are unchanged.
