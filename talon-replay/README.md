# Talon FlightLab

Present and run [Talon](..) UAV autopilot simulations in the browser. One
entry point carries four sections:

- **Replay**: a full-bleed 3D scene. Scrub a quadrotor's recorded flight and
  read the autopilot working in real time: EKF state estimation (with a
  3-sigma covariance ellipsoid), the flight-phase state machine (colored
  trail and phase-segmented timeline), NLGL waypoint guidance, and
  deterministic detour avoidance around spherical keep-out zones (live
  surface clearance in the HUD).
- **Telemetry**: the recorded channels as time-series charts (altitude,
  ground speed, EKF error, wind, thrust, 3-sigma, obstacle clearance), all
  sharing the replay clock.
- **Metrics**: a run summary derived from the active scenario plus the
  validation driver's own acceptance table.
- **Videos**: the recorded Matplotlib flight exports, with an "open in
  replay" jump for the runs that have a scenario.

Built with React 19, Vite 6, TypeScript 5.7, react-three-fiber 9, Zustand 5,
and a restrained bloom pass via `@react-three/postprocessing`. Charts are
hand-rolled SVG, so there is no charting dependency.

The live app lives under `/talon-replay/` (vite `base`), so the repository
directory keeps its original name.

## Run it

```bash
npm install
npm run dev
```

Then open the printed URL.

```bash
npm run build    # typecheck + production build into dist/
npm test         # vitest unit tests (interpolation, store, charts, telemetry)
```

## Scenarios

Telemetry is exported from the Talon simulator as schema 1.1 JSON and served
from `public/data/`:

| File              | Story                                                       |
| ----------------- | ----------------------------------------------------------- |
| `gps_outage`      | Full waypoint mission with a GPS-denied segment mid-flight   |
| `wind`            | Altitude hold in a ~2 m/s gust, exercising wind estimation   |
| `emergency`       | Critical fault at t=20 s; EKF diverges (covariance blows up) |
| `obstacle_field`  | Rectangle circuit through 3 spherical keep-out zones; the    |
|                   | planner inserts detour waypoints and the flight clears them  |
|                   | by 2.07 m (schema 1.1 `obstacles`)                           |

Schema 1.1 adds the top-level `obstacles` list
(`[{ id, center_ned, radius_m }]`). It is backward compatible: a 1.0 document
without the key loads as an empty list.

## Refresh the data

1. Run the Talon simulations so `../plots/json/*.json` (including
   `validation.json`) and `../docs/videos/*.mp4` are regenerated (from the
   Talon repo root: `uv run python sims/run_all.py`).
2. From this directory:

```bash
scripts/copy_data.sh
```

This copies the four scenario JSONs plus `validation.json` into
`public/data/` and the five recordings into `public/videos/`.

## Live demo

<!-- Replace with the real Pages URL once the workflow has deployed. The
     final URL depends on the repo name: with vite base /talon-replay/
     deployed from the Talon repo's Pages, it resolves under
     https://<user>.github.io/<repo>/talon-replay/. -->
https://ajeet.github.io/Talon/talon-replay/

## Layout

- Header: wordmark plus the section tabs (Replay, Telemetry, Metrics,
  Videos).
- Left rail: scenario selector, plus display toggles on the time-based
  sections; the keep-out toggle appears only when the scenario ships
  obstacles.
- Replay: phase-colored truth trail, quadrotor marker, EKF covariance
  ellipsoid, wind particles, numbered waypoint spheres, and spherical
  keep-out zones (translucent fill, wireframe shell, ground footprint, id +
  radius label) with a restrained bloom pass on the emissive accents.
- HUD: mission clock, live phase chip, altitude, ground speed, and, for the
  obstacle scenario, live surface clearance to the nearest zone (amber clear,
  yellow caution, rose breach) named by the zone.
- Legend: flight-phase key plus a keep-out inventory with the flight-minimum
  surface clearance.
- Telemetry: one chart per channel on a shared time axis; the cursor and the
  live readouts ride the replay clock.
- Metrics: client-derived run summary plus the driver's validation gates
  (`validation.json`), so the web report and the printed report cannot drift.
- Videos: the recorded MP4s with an inline player.
- Bottom bar: transport (play/pause, 0.5x-4x) and the phase-segmented
  scrubber, pinned for the Replay and Telemetry sections.

## Source map

- `src/views/`: the four sections.
- `src/components/scene/`: R3F layers (scene composition in FlightScene.tsx).
- `src/components/charts/`: the SVG time-series chart.
- `src/components/ui/`: rail, tabs, HUD, legend, timeline, stat tiles, panels.
- `src/utils/`: `interpolation.ts`, `clearance.ts`, `chart.ts`, `telemetry.ts`,
  `colors.ts`, `format.ts` (all pure and unit-tested).
- `src/store/useFlightStore.ts`: scenario cache, playback clock, toggles,
  active section, validation report.
- `src/data/`: scenario and recording metadata.
