# Talon Flight Replay

Interactive 3D replay of [Talon](..) UAV autopilot simulations. Scrub a
quadrotor's recorded flight in the browser and read the autopilot working in
real time: EKF state estimation (with a 3-sigma covariance ellipsoid), the
flight-phase state machine (colored trail and phase-segmented timeline),
NLGL waypoint guidance, and deterministic detour avoidance around spherical
keep-out zones (live surface clearance in the HUD).

Built with React 19, Vite 6, TypeScript 5.7, react-three-fiber 9, and a
restrained bloom pass via `@react-three/postprocessing`.

## Run it

```bash
npm install
npm run dev
```

Then open the printed URL (the app is served under `/talon-replay/`).

```bash
npm run build    # typecheck + production build into dist/
npm test         # vitest unit tests (interpolation + store)
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

1. Run the Talon simulations so `../plots/json/*.json` is regenerated
   (from the Talon repo root: `uv run python sims/run_all.py`).
2. From this directory:

```bash
scripts/copy_data.sh
```

This copies `gps_outage.json`, `wind.json`, `emergency.json`, and
`obstacle_field.json` into `public/data/`.

## Live demo

<!-- Replace with the real Pages URL once the workflow has deployed. The
     final URL depends on the repo name: with vite base /talon-replay/
     deployed from the Talon repo's Pages, it resolves under
     https://<user>.github.io/<repo>/talon-replay/. -->
https://ajeet.github.io/Talon/talon-replay/

## Layout

- Full-bleed 3D scene: phase-colored truth trail, quadrotor marker, EKF
  covariance ellipsoid, wind particles, numbered waypoint spheres, and
  spherical keep-out zones (translucent fill, wireframe shell, ground
  footprint, id + radius label) with a restrained bloom pass on the emissive
  accents.
- Left rail: scenario selector (including OBSTACLE FIELD) and display toggles;
  the keep-out toggle appears only when the scenario ships obstacles.
- HUD: mission clock, live phase chip, altitude, ground speed, and, for the
  obstacle scenario, live surface clearance to the nearest zone (amber clear,
  yellow caution, rose breach) named by the zone.
- Legend: flight-phase key plus a keep-out inventory with the flight-minimum
  surface clearance.
- Bottom bar: transport (play/pause, 0.5x-4x) and the phase-segmented
  scrubber; the live phase chip in the HUD changes color as you scrub.
