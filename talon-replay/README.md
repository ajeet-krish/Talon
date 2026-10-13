# Talon Flight Replay

Interactive 3D replay of [Talon](..) UAV autopilot simulations. Scrub a
quadrotor's recorded flight in the browser and read the autopilot working in
real time: EKF state estimation (with a 3-sigma covariance ellipsoid), the
flight-phase state machine (colored trail and phase-segmented timeline), and
NLGL waypoint guidance.

Built with React 19, Vite 6, TypeScript 5.7, and react-three-fiber 9.

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

Telemetry is exported from the Talon simulator as schema 1.0 JSON and served
from `public/data/`:

| File            | Story                                                        |
| --------------- | ------------------------------------------------------------ |
| `gps_outage`    | Full waypoint mission with a GPS-denied segment mid-flight    |
| `wind`          | Altitude hold in a ~2 m/s gust, exercising wind estimation    |
| `emergency`     | Critical fault at t=20 s; EKF diverges (covariance blows up)  |

## Refresh the data

1. Run the Talon simulations so `../plots/json/*.json` is regenerated
   (from the Talon repo root: `uv run python sims/run_all.py`).
2. From this directory:

```bash
scripts/copy_data.sh
```

This copies `gps_outage.json`, `wind.json`, and `emergency.json` into
`public/data/`.

## Live demo

<!-- Replace with the real Pages URL once the workflow has deployed. The
     final URL depends on the repo name: with vite base /talon-replay/
     deployed from the Talon repo's Pages, it resolves under
     https://<user>.github.io/<repo>/talon-replay/. -->
https://ajeet.github.io/Talon/talon-replay/

## Layout

- Full-bleed 3D scene: phase-colored truth trail, quadrotor marker, EKF
  covariance ellipsoid, wind particles, numbered waypoint spheres.
- Left rail: scenario selector and display toggles.
- Bottom bar: transport (play/pause, 0.5x-4x) and the phase-segmented
  scrubber; the live phase chip in the HUD changes color as you scrub.
