from __future__ import annotations

import pytest

from talon import BatteryConfig, BatteryModel


def test_hover_drain_matches_coulomb_counting() -> None:
    # 60 s at hover thrust: I = hover_current = 5 A,
    # dAh = 5 * 60 / 3600 = 1/12 Ah, soc delta = (1/12) / 1.5 Ah.
    battery = BatteryModel()
    state = battery.update(thrust_n=14.715, dt=60.0)
    expected = 1.0 - (5.0 * 60.0 / 3600.0) / 1.5
    assert state.current_a == pytest.approx(5.0)
    assert battery.soc == pytest.approx(expected)
    assert battery.soc == pytest.approx(1.0 - 0.0555555, abs=1e-6)


def test_current_scales_with_thrust() -> None:
    battery = BatteryModel()
    state = battery.update(thrust_n=2.0 * 14.715, dt=0.1)
    assert state.current_a == pytest.approx(10.0)


def test_is_low_and_is_critical_thresholds() -> None:
    battery = BatteryModel()
    battery.force_soc(0.21)
    assert not battery.is_low
    battery.force_soc(0.19)
    assert battery.is_low
    assert not battery.is_critical
    battery.force_soc(0.09)
    assert battery.is_critical


def test_force_soc_clamps_and_overrides() -> None:
    battery = BatteryModel(initial_soc=1.0)
    battery.force_soc(0.5)
    assert battery.soc == 0.5
    battery.force_soc(1.7)
    assert battery.soc == 1.0
    battery.force_soc(-0.3)
    assert battery.soc == 0.0


def test_voltage_drops_under_load() -> None:
    # Same soc, heavier load -> bigger IR drop.
    battery_light = BatteryModel(initial_soc=0.8)
    battery_heavy = BatteryModel(initial_soc=0.8)
    light = battery_light.update(thrust_n=14.715, dt=1.0)
    heavy = battery_heavy.update(thrust_n=2.0 * 14.715, dt=1.0)
    assert heavy.voltage < light.voltage
    # Open-circuit-style term at soc = 0.8 minus IR drop at 10 A.
    cfg = BatteryConfig()
    expected_heavy = cfg.nominal_voltage * (0.9 + 0.1 * 0.8) - 10.0 * cfg.internal_resistance
    assert heavy.voltage == pytest.approx(expected_heavy, abs=0.01)


def test_zero_thrust_draws_idle_floor() -> None:
    battery = BatteryModel()
    state = battery.update(thrust_n=0.0, dt=3600.0)
    cfg = BatteryConfig()
    assert state.current_a == cfg.idle_current_a
    # 1 h at 0.2 A against a 1.5 Ah pack: 0.2 / 1.5 = 0.1333 soc drained.
    assert battery.soc == pytest.approx(1.0 - 0.2 / 1.5)


def test_custom_config_and_initial_soc() -> None:
    cfg = BatteryConfig(capacity_mah=3000.0, hover_current_a=10.0)
    battery = BatteryModel(cfg, initial_soc=0.5)
    state = battery.update(thrust_n=14.715, dt=3600.0)
    assert state.current_a == pytest.approx(10.0)
    # 10 A for 1 h against a 3 Ah pack: drains past empty, floors at 0.
    assert battery.soc == 0.0


def test_invalid_config_rejected() -> None:
    with pytest.raises(ValueError):
        BatteryConfig(capacity_mah=0.0)
    with pytest.raises(ValueError):
        BatteryConfig(hover_thrust_n=0.0)
    battery = BatteryModel()
    with pytest.raises(ValueError):
        battery.update(thrust_n=14.715, dt=-1.0)
