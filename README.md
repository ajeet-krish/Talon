# Talon

Talon is a single-vehicle UAV autopilot for a 1.5 kg Crazyflie-class
quadrotor: a full guidance, navigation, and control stack with a header-only
C++17 computational kernel (rigid-body dynamics, Dryden wind, later controller,
EKF, and NLGL guidance) exposed to Python through pybind11, plus Python-side
tooling for motors, simulation, and analysis. Phase 1 ships the dynamics
kernel, wind model, motor mixing with lag, and the Python bindings.

## Build and test

```bash
cmake -B build && cmake --build build -j   # builds talon_tests and src/talon/_core*.so
cd build && ./talon_tests                  # C++ unit tests
cd .. && uv sync                           # Python environment
uv run pytest tests/                       # Python tests
```

Run CMake from `uv run` (or with the virtualenv active) so the extension is
built for the same interpreter that runs the tests.

Frame, sign, and unit conventions live in [docs/conventions.md](docs/conventions.md).
A full technical-report style README arrives in Phase 6.
