# Talon flight videos

Cinematic 3D flight exports (Sprint 2.4). Every video is rendered from a
full SIL history by the validation driver:

```sh
uv run python sims/run_all.py
```

Use `--skip-video` (or set `TALON_SKIP_VIDEO=1`) to skip the export. The
driver writes five MP4s here (libx264, 30 fps, 2x time compression,
120 dpi): `gps_outage.mp4`, `wind.mp4`, `emergency.mp4`, `waypoints.mp4`,
and `obstacle_field.mp4`. Each video shows the phase-colored trajectory
trail, the quadrotor marker with a velocity-heading cross, a ground shadow,
numbered waypoint pins, a monospace HUD (time, phase, altitude, speed),
and a fade-in title card. When ffmpeg is unavailable the exporter
degrades to a PillowWriter GIF (path suffix adjusted to `.gif`).

The five recordings are also surfaced inside the FlightLab tool's Videos
section; `talon-replay/scripts/copy_data.sh` stages them into
`talon-replay/public/videos/`.

## Waypoint circuit

<video src="waypoints.mp4" controls width="640"></video>

[waypoints.mp4](waypoints.mp4)

## Obstacle field

<video src="obstacle_field.mp4" controls width="640"></video>

[obstacle_field.mp4](obstacle_field.mp4)
