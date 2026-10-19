"""Python vs ``talon._core`` parity for the Coulomb-counting battery model.

The config defaults, one-step and 100-step drain, the strict low and
critical thresholds, the ``force_soc`` clamp and the constructor error
texts are exercised on both implementations, plus five registered
divergences: the negative and NaN ``dt`` policies, the NaN config
acceptance, the NaN soc clamp, and the config snapshot policy. Config
validation lives in
``BatteryConfig.__post_init__`` on the Python side but in
``BatteryModel::new`` on the Rust side, so the error-parity tests compare
Python ``BatteryConfig(...)`` against ``_core.BatteryModel(...)``.
"""

from __future__ import annotations

import math

import pytest

from parity_util import assert_exact, assert_near, assert_same_error, expect_divergence
from talon import _core
from talon.battery import BatteryConfig, BatteryModel

HOVER_THRUST = 14.715


def models(
    config: BatteryConfig | None = None,
    rs_config: "_core.BatteryConfig | None" = None,
    initial_soc: float = 1.0,
) -> tuple[BatteryModel, "_core.BatteryModel"]:
    return BatteryModel(config, initial_soc), _core.BatteryModel(rs_config, initial_soc)


# ---------------------------------------------------------------------------
# Config defaults, constructor and setters
# ---------------------------------------------------------------------------


def test_config_defaults_parity() -> None:
    py_cfg, rs_cfg = BatteryConfig(), _core.BatteryConfig()
    assert_exact(rs_cfg.capacity_mah, py_cfg.capacity_mah)
    assert_exact(rs_cfg.nominal_voltage, py_cfg.nominal_voltage)
    assert_exact(rs_cfg.hover_current_a, py_cfg.hover_current_a)
    assert_exact(rs_cfg.internal_resistance, py_cfg.internal_resistance)
    assert_exact(rs_cfg.idle_current_a, py_cfg.idle_current_a)
    # Near-bit tier rather than assert_exact for hover_thrust_n: the
    # Python dataclass defaults to the literal 14.715 while the Rust core
    # computes MASS * GRAVITY (core/src/battery.rs). The two doubles
    # happen to be bit-identical today, but the policy records that one
    # side writes a literal and the other a product.
    assert_near(rs_cfg.hover_thrust_n, py_cfg.hover_thrust_n)
    assert_exact(py_cfg.capacity_mah, 1500.0)
    assert_exact(py_cfg.nominal_voltage, 11.1)
    assert_exact(py_cfg.hover_current_a, 5.0)
    assert_exact(py_cfg.internal_resistance, 0.05)
    assert_exact(py_cfg.idle_current_a, 0.2)


def test_config_constructor_and_setter_parity() -> None:
    py_cfg = BatteryConfig(capacity_mah=3000.0, hover_current_a=10.0)
    rs_cfg = _core.BatteryConfig(capacity_mah=3000.0, hover_current_a=10.0)
    assert_exact(rs_cfg.capacity_mah, py_cfg.capacity_mah)
    assert_exact(rs_cfg.hover_current_a, py_cfg.hover_current_a)
    # Unset fields keep the defaults on both sides.
    assert_exact(rs_cfg.nominal_voltage, py_cfg.nominal_voltage)

    py_cfg.idle_current_a = 0.5
    rs_cfg.idle_current_a = 0.5
    py_cfg.internal_resistance = 0.02
    rs_cfg.internal_resistance = 0.02
    assert_exact(rs_cfg.idle_current_a, py_cfg.idle_current_a)
    assert_exact(rs_cfg.internal_resistance, py_cfg.internal_resistance)


# ---------------------------------------------------------------------------
# Drain behaviour
# ---------------------------------------------------------------------------


def test_one_step_drain_parity() -> None:
    # One hover step: current is the hover current and the charge tick is
    # I * dt / 3600 against a 1.5 Ah pack.
    py_model, rs_model = models()
    py_cfg, rs_cfg = BatteryConfig(), _core.BatteryConfig()
    py_state = py_model.update(py_cfg.hover_thrust_n, 0.005)
    rs_state = rs_model.update(rs_cfg.hover_thrust_n, 0.005)
    assert_near(py_state.current_a, 5.0, label="python hover current")
    assert_near(rs_state.current_a, 5.0, label="rust hover current")
    assert_near(rs_state.current_a, py_state.current_a, label="current parity")
    expected_soc = 1.0 - (5.0 * 0.005 / 3600.0) / 1.5
    assert_near(py_state.soc, expected_soc, label="python soc step")
    assert_near(rs_state.soc, expected_soc, label="rust soc step")
    assert_near(rs_state.soc, py_state.soc, label="soc parity")
    assert_near(rs_model.soc, py_model.soc, label="model soc parity")
    # The voltage is the linear droop minus the IR drop on both sides.
    expected_voltage = 11.1 * (0.9 + 0.1 * expected_soc) - 5.0 * 0.05
    assert_near(py_state.voltage, expected_voltage, label="python voltage")
    assert_near(rs_state.voltage, expected_voltage, label="rust voltage")
    assert_near(rs_state.voltage, py_state.voltage, label="voltage parity")


def test_hundred_step_drain_parity() -> None:
    # 100 hover steps of 0.1 s = 10 s of flight; the two models must track
    # each other to the parity tolerance at every comparison point.
    py_model, rs_model = models()
    py_cfg, rs_cfg = BatteryConfig(), _core.BatteryConfig()
    for step in range(100):
        py_state = py_model.update(py_cfg.hover_thrust_n, 0.1)
        rs_state = rs_model.update(rs_cfg.hover_thrust_n, 0.1)
        if step % 25 == 0:
            assert_near(rs_state.soc, py_state.soc, label=f"soc parity at step {step}")
            assert_near(rs_state.current_a, py_state.current_a, label="current parity")
    expected_soc = 1.0 - (5.0 * 10.0 / 3600.0) / 1.5
    assert_near(py_state.soc, expected_soc, label="python final soc")
    assert_near(rs_state.soc, expected_soc, label="rust final soc")
    assert_near(rs_state.voltage, py_state.voltage, label="final voltage parity")


@pytest.mark.parametrize(
    "thrust",
    [
        pytest.param(0.0, id="zero_thrust_idle_floor"),
        pytest.param(-50.0, id="negative_thrust_clamped"),
    ],
)
def test_idle_floor_parity(thrust: float) -> None:
    # A non-positive thrust draws only the idle floor, exactly, and the
    # pack still drains.
    py_model, rs_model = models()
    py_state = py_model.update(thrust, 60.0)
    rs_state = rs_model.update(thrust, 60.0)
    assert_exact(py_state.current_a, 0.2)
    assert_exact(rs_state.current_a, 0.2)
    assert_exact(rs_state.current_a, py_state.current_a, label="idle current parity")
    expected_soc = 1.0 - (0.2 * 60.0 / 3600.0) / 1.5
    assert_near(py_state.soc, expected_soc, label="python idle soc")
    assert_near(rs_state.soc, expected_soc, label="rust idle soc")
    assert_near(rs_state.soc, py_state.soc, label="idle soc parity")


def test_voltage_droop_parity() -> None:
    # Same soc, heavier load: bigger IR drop, on both sides.
    py_light, rs_light = models(initial_soc=0.8)
    py_heavy, rs_heavy = models(initial_soc=0.8)
    py_cfg, rs_cfg = BatteryConfig(), _core.BatteryConfig()
    py_light_state = py_light.update(py_cfg.hover_thrust_n, 1.0)
    rs_light_state = rs_light.update(rs_cfg.hover_thrust_n, 1.0)
    py_heavy_state = py_heavy.update(2.0 * py_cfg.hover_thrust_n, 1.0)
    rs_heavy_state = rs_heavy.update(2.0 * rs_cfg.hover_thrust_n, 1.0)
    assert py_heavy_state.voltage < py_light_state.voltage
    assert rs_heavy_state.voltage < rs_light_state.voltage
    assert_near(rs_light_state.voltage, py_light_state.voltage, label="light parity")
    assert_near(rs_heavy_state.voltage, py_heavy_state.voltage, label="heavy parity")
    assert_near(py_heavy_state.current_a, 10.0, label="python double-current")
    assert_near(rs_heavy_state.current_a, 10.0, label="rust double-current")


# ---------------------------------------------------------------------------
# Thresholds, force_soc and the constructor clamp
# ---------------------------------------------------------------------------


@pytest.mark.parametrize(
    ("soc", "low", "critical"),
    [
        pytest.param(0.20, False, False, id="at_low_threshold"),
        pytest.param(0.19, True, False, id="below_low_threshold"),
        pytest.param(0.10, True, False, id="at_critical_threshold"),
        pytest.param(0.099, True, True, id="below_critical_threshold"),
    ],
)
def test_low_and_critical_boundaries_parity(soc: float, low: bool, critical: bool) -> None:
    # Both thresholds are strict inequalities on both sides.
    py_model, rs_model = models()
    py_model.force_soc(soc)
    rs_model.force_soc(soc)
    assert py_model.is_low is low
    assert rs_model.is_low is low
    assert py_model.is_critical is critical
    assert rs_model.is_critical is critical
    assert_exact(rs_model.soc, py_model.soc, label="forced soc parity")


@pytest.mark.parametrize(
    ("soc", "clamped"),
    [
        pytest.param(1.7, 1.0, id="above_one"),
        pytest.param(-0.3, 0.0, id="below_zero"),
        pytest.param(0.5, 0.5, id="inside"),
    ],
)
def test_force_soc_clamp_parity(soc: float, clamped: float) -> None:
    py_model, rs_model = models()
    py_model.force_soc(soc)
    rs_model.force_soc(soc)
    assert_exact(py_model.soc, clamped, label="python clamp")
    assert_exact(rs_model.soc, clamped, label="rust clamp")
    assert_exact(rs_model.soc, py_model.soc, label="clamp parity")


@pytest.mark.parametrize(
    ("initial_soc", "clamped"),
    [
        pytest.param(5.0, 1.0, id="above_one"),
        pytest.param(-3.0, 0.0, id="below_zero"),
        pytest.param(0.4, 0.4, id="inside"),
    ],
)
def test_initial_soc_clamp_parity(initial_soc: float, clamped: float) -> None:
    py_model, rs_model = models(initial_soc=initial_soc)
    assert_exact(py_model.soc, clamped, label="python initial clamp")
    assert_exact(rs_model.soc, clamped, label="rust initial clamp")
    assert_exact(rs_model.soc, py_model.soc, label="initial clamp parity")


def test_default_model_parity() -> None:
    py_model, rs_model = models()
    assert_exact(py_model.soc, 1.0)
    assert_exact(rs_model.soc, py_model.soc, label="default soc parity")
    assert py_model.is_low is False and rs_model.is_low is False
    assert py_model.is_critical is False and rs_model.is_critical is False


def test_rust_only_current_accessor() -> None:
    # current_a mirrors the Rust core surface; the Python model has no
    # accessor, so only the binding side is asserted here.
    rs_model = _core.BatteryModel()
    assert_exact(rs_model.current_a, 0.0)
    rs_state = rs_model.update(HOVER_THRUST, 0.005)
    assert_exact(rs_model.current_a, rs_state.current_a)


# ---------------------------------------------------------------------------
# Constructor validation
# ---------------------------------------------------------------------------


@pytest.mark.parametrize(
    ("kwargs", "field", "value"),
    [
        pytest.param({"capacity_mah": 0.0}, "capacity_mah", 0.0, id="zero_capacity"),
        pytest.param({"capacity_mah": -1.0}, "capacity_mah", -1.0, id="negative_capacity"),
        pytest.param({"hover_thrust_n": 0.0}, "hover_thrust_n", 0.0, id="zero_hover_thrust"),
        pytest.param({"hover_thrust_n": -2.0}, "hover_thrust_n", -2.0, id="negative_hover_thrust"),
    ],
)
def test_config_error_parity(kwargs: dict, field: str, value: float) -> None:
    # The Python dataclass validates in __post_init__; the binding
    # validates in BatteryModel's constructor, where the Rust core does.
    message = f"{field} must be positive, got {value}"
    with pytest.raises(ValueError) as py_error:
        BatteryConfig(**kwargs)
    with pytest.raises(ValueError) as rust_error:
        _core.BatteryModel(_core.BatteryConfig(**kwargs))
    assert_same_error(py_error.value, rust_error.value, label=f"config {field} {value}")
    assert str(rust_error.value) == message


# ---------------------------------------------------------------------------
# Registered divergences
# ---------------------------------------------------------------------------


def test_negative_dt_is_a_registered_divergence() -> None:
    reason = expect_divergence("battery.update_negative_dt")
    py_model, rs_model = models()
    with pytest.raises(ValueError, match="dt must be non-negative"):
        py_model.update(HOVER_THRUST, -1.0)
    # The Rust core clamps dt to zero: the state of charge is unchanged,
    # and the current and voltage are still reported.
    rs_state = rs_model.update(HOVER_THRUST, -1.0)
    assert_exact(rs_model.soc, 1.0, label="rust soc unchanged")
    assert_near(rs_state.current_a, 5.0, label="rust current still reported")
    assert reason.startswith("Python BatteryModel.update raises ValueError")


def test_nan_dt_is_a_registered_divergence() -> None:
    reason = expect_divergence("battery.update_nan_dt")
    # Python's update guard is `dt < 0.0`, and NaN comparisons are all
    # False, so a NaN dt integrates a NaN charge step; max(0.0, nan)
    # swallows it and the state of charge floors to 0.0 after one call.
    py_model, rs_model = models()
    py_state = py_model.update(HOVER_THRUST, float("nan"))
    assert_exact(py_state.soc, 0.0, label="python NaN dt floors soc")
    assert_near(py_state.current_a, 5.0, label="python current still reported")
    assert_exact(py_model.soc, 0.0, label="python model soc poisoned")
    # The Rust core treats a NaN dt like a negative one (`NaN > 0.0` is
    # False): clamped to zero, so the state of charge is unchanged and
    # the voltage matches an explicit zero-length step.
    rs_state = rs_model.update(HOVER_THRUST, float("nan"))
    zero_dt_state = rs_model.update(HOVER_THRUST, 0.0)
    assert_exact(rs_model.soc, 1.0, label="rust soc unchanged")
    assert_exact(rs_state.soc, zero_dt_state.soc, label="rust NaN equals dt=0")
    assert_exact(rs_state.voltage, zero_dt_state.voltage, label="rust voltage parity")
    assert_near(rs_state.current_a, 5.0, label="rust current still reported")
    assert reason.startswith("Python BatteryModel.update guards only dt < 0.0")


def test_nan_config_is_a_registered_divergence() -> None:
    reason = expect_divergence("battery.config_nan_rejected")
    # Python's guards are `<= 0.0`, and NaN comparisons are all False, so
    # a NaN capacity or hover thrust is accepted and silently corrupts the
    # model: max(0.0, nan) swallows the NaN, so a NaN capacity floors the
    # state of charge to 0.0 after one step and a NaN hover thrust pins
    # the current at the idle floor.
    py_model = BatteryModel(BatteryConfig(capacity_mah=float("nan")))
    py_state = py_model.update(1.0, 1.0)
    assert_exact(py_state.soc, 0.0, label="python NaN capacity floors soc")
    py_model = BatteryModel(BatteryConfig(hover_thrust_n=float("nan")))
    py_state = py_model.update(1.0, 1.0)
    assert_exact(py_state.current_a, 0.2, label="python NaN thrust pins idle")
    # The Rust core rejects NaN explicitly (core/src/battery.rs).
    with pytest.raises(ValueError, match="capacity_mah must be positive"):
        _core.BatteryModel(_core.BatteryConfig(capacity_mah=float("nan")))
    with pytest.raises(ValueError, match="hover_thrust_n must be positive"):
        _core.BatteryModel(_core.BatteryConfig(hover_thrust_n=float("nan")))
    assert reason.startswith("Python BatteryConfig(nan) is accepted")


def test_nan_soc_clamp_is_a_registered_divergence() -> None:
    reason = expect_divergence("battery.soc_nan_clamp")
    # Python clamps with min(1.0, max(0.0, soc)); max(0.0, nan) returns
    # 0.0 (the first argument, since the comparison is False), so the
    # NaN is swallowed and the pack reads empty instead of corrupted.
    py_model = BatteryModel(initial_soc=float("nan"))
    assert_exact(py_model.soc, 0.0, label="python initial_soc nan floors to 0.0")
    py_force = BatteryModel()
    py_force.force_soc(float("nan"))
    assert_exact(py_force.soc, 0.0, label="python force_soc nan floors to 0.0")
    # Rust f64::clamp returns NaN for a NaN self, so the _core keeps the
    # corrupted value visible.
    rs_model = _core.BatteryModel(initial_soc=float("nan"))
    assert math.isnan(rs_model.soc), rs_model.soc
    rs_force = _core.BatteryModel()
    rs_force.force_soc(float("nan"))
    assert math.isnan(rs_force.soc), rs_force.soc
    assert reason.startswith("Python BatteryModel clamps a NaN initial_soc")


def test_config_is_a_snapshot_divergence() -> None:
    reason = expect_divergence("battery.config_is_a_snapshot")
    # Python re-reads the passed config object on every update; the
    # binding copies it at construction (the same snapshot policy as
    # NLGLGuidance.config), so the post-construction write only reaches
    # the Python model.
    py_cfg, rs_cfg = BatteryConfig(), _core.BatteryConfig()
    py_model, rs_model = models(py_cfg, rs_cfg)
    py_cfg.hover_current_a = 10.0
    rs_cfg.hover_current_a = 10.0
    py_state = py_model.update(py_cfg.hover_thrust_n, 1.0)
    rs_state = rs_model.update(rs_cfg.hover_thrust_n, 1.0)
    assert_near(py_state.current_a, 10.0, label="python follows the write")
    assert_near(rs_state.current_a, 5.0, label="rust snapshot keeps 5 A")
    assert reason.startswith("Python BatteryModel keeps the passed BatteryConfig")
