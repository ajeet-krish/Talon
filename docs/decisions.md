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

## Open decisions (Phases 2-4)

- D7: Acceptance metrics for v2 reactive avoidance (min clearance
  margin, allowed e_ct degradation, Monte Carlo min-clearance stats).
- D8: Visual identity: how far toward SwarmGNC's bloom aesthetic versus
  the current instrument look.
- D9: WASM boundary: which Python glue functions are ported to TS
  versus trimmed for the sandbox.
