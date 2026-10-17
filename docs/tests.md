# Talon Test Results

Last updated: 2026-10-17

## Repository restructure (2026-10-17)

The web app moved to the repository root, the C++ core to `core/`, and the
Python package, simulations, and tests to `python/`. Every suite was re-run at
the new paths; the only code changes were path configuration.

| Suite | Command | Result |
|-------|---------|--------|
| C++ kernel (Google Test) | `./build/talon_tests` | 34 pass |
| Python (pytest) | `uv run pytest python/tests/ -q` | 186 pass |
| Config + store + utils (vitest) | `npm test` (repo root) | 60 pass |
| Validation driver | `uv run python python/sims/run_all.py --skip-video --skip-fingerprint` | 25/25 checks, 7/7 scenario asserts |
| Typecheck + production build | `npm run build` | clean |

Path-only changes: `CMakeLists.txt` (`core/include`, `core/tests`, and the
`python/talon` extension output dir), `pyproject.toml` (packages, testpaths,
pythonpath), `python/sims/run_all.py` (`PY_ROOT`/`REPO_ROOT`, pytest path),
`.github/workflows/ci.yml` (pytest path plus a new web job), and
`scripts/copy_data.sh` (repository-root paths).

## FlightLab Phase 3: unified entry point (2026-10-17)

| Suite | Command | Result |
|-------|---------|--------|
| C++ kernel (Google Test) | `./build/talon_tests` | 34 pass |
| Python (pytest) | `uv run pytest python/tests/ -q` | 186 pass |
| Config + store + utils (vitest) | `npm test` (repo root) | 60 pass |
| Validation driver | `uv run python python/sims/run_all.py --skip-fingerprint` | 25/25 checks, 7/7 scenario asserts |
| Typecheck | `npm run typecheck` | clean |
| Production build | `npm run build` | clean, 629 modules |

### New frontend tests (Phase 3)

- `src/utils/chart.test.ts` (10): decimation returns a copy when the series
  fits, reduces below the budget while keeping time order, and preserves a
  lone spike; `valueDomain` pads normal ranges, gives a flat series height,
  and falls back for an empty input; `xScale`/`yScale` hit the plot edges and
  invert the value axis; `buildPath` is empty with no samples and starts with
  a move plus a line otherwise.
- `src/utils/telemetry.test.ts` (6): `deriveTelemetry` derives altitude,
  speed, EKF error, and 3-sigma from the samples, flags a zero wind channel
  as flat, omits the clearance series without keep-out zones, and adds it
  (with correct surface distances) when zones are present; `summarizeRun`
  reports duration, rate, phase count, terminal phase, EKF RMSE/max, maxima,
  distance flown, and minimum clearance.
- `src/utils/format.test.ts` (4): precision tracks magnitude, zero is plain,
  non-finite input renders `--`, and negatives keep their sign.
- `src/store/useFlightStore.test.ts` (3 added): `setView` switches sections;
  `loadValidation` caches a successful report and records an error status on
  a failed fetch.

### Regeneration verification

- `python/sims/run_all.py` regenerated `plots/json/*.json` plus `plots/json/validation.json`
  and `docs/videos/*.mp4`. The four previously committed MP4s were reproduced
  byte-for-byte (`git diff --stat -- docs/videos/` is empty); only the new
  `obstacle_field.mp4` was added.
- `scripts/copy_data.sh` staged five JSON documents into `public/data/` and
  five recordings into `public/videos/`; the production build copies both
  trees into `dist/`.
