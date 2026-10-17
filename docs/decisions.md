# Talon Decisions

Last updated: 2026-10-17

## FlightLab visual tool (2026-10-16)

### D1: Extend talon-replay in place, no new app

**Decision.** The interactive visual tool evolves `talon-replay/` into a
unified present-and-run experience. No separate web app.

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

### D4: Schema 1.1 with optional obstacles, backward compatible

**Decision.** `SCHEMA_VERSION = "1.1"`. The `obstacles` top-level key
is written on every export. Old 1.0 documents (without the key) remain
valid; the validator checks the block only when present.

**Rationale.** The exporter is the single source of truth for the JSON
contract. Keeping the validator lenient on the new key means existing
`plots/json/*.json` files and the talon-replay TS types do not break on
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
`talon-replay/public/videos/` and committed, mirroring how `plots/json/*.json`
is copied into `public/data/`.

**Rationale.** The recordings are small (about 1.6 MB total) and static
hosting must serve them from `dist`. Committing keeps the app self-contained,
so a build never depends on the monorepo layout or a prebuild copy step.

**Trade-off.** The MP4 bytes exist twice in the repository. Acceptable at
this size; regenerate with `copy_data.sh` after `run_all.py`.

## Open decisions (Phase 4)

- Acceptance metrics for v2 reactive avoidance (min clearance margin,
  allowed e_ct degradation, Monte Carlo min-clearance stats).
- WASM boundary: which Python glue functions are ported to TS versus
  trimmed for the sandbox.
- Whether the live WASM source reuses the recorded `FlightDataSource`
  interface or streams frames into a ring buffer first.
