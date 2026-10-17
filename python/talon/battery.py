"""Battery model (Phase 4).

Simple Coulomb-counting model with a thrust-proportional load:

    I       = max(idle_current, hover_current * thrust / hover_thrust)
    dAh     = I * dt / 3600
    soc    -= dAh / capacity_mah
    voltage = nominal_voltage * (0.9 + 0.1 * soc) - I * internal_resistance

Documented model choices:

- Current scales linearly with collective thrust (motor electrical power
  ~ torque * speed, both ~ thrust for a fixed-pitch prop; the linear map
  is the standard first-order SIL approximation). A small idle floor
  keeps the battery draining (slowly) at zero thrust, like real
  avionics + idle ESCs.
- Voltage is an open-circuit-vs-soc linear droop (100% at full soc, 90%
  at empty) minus an IR ohmic drop under load. No hysteresis, no
  temperature: adequate for RTL-trigger logic, not for power design.
- 1500 mAh at ~5 A hover gives ~18 min of hover endurance. Scenarios
  inject low battery with force_soc() rather than draining organically.
"""

from __future__ import annotations

from dataclasses import dataclass


@dataclass(slots=True)
class BatteryConfig:
    """Battery electrical / load parameters (SI, mAh for capacity)."""

    capacity_mah: float = 1500.0
    nominal_voltage: float = 11.1
    hover_current_a: float = 5.0
    hover_thrust_n: float = 14.715  # mg (matches talon.hover_thrust)
    internal_resistance: float = 0.05
    idle_current_a: float = 0.2

    def __post_init__(self) -> None:
        if self.capacity_mah <= 0.0:
            raise ValueError(f"capacity_mah must be positive, got {self.capacity_mah}")
        if self.hover_thrust_n <= 0.0:
            raise ValueError(
                f"hover_thrust_n must be positive, got {self.hover_thrust_n}"
            )


@dataclass(slots=True)
class BatteryState:
    """One battery sample: state of charge (0..1), voltage (V), current (A)."""

    soc: float
    voltage: float
    current_a: float


class BatteryModel:
    """Coulomb-counting battery with thrust-proportional load and IR drop."""

    def __init__(
        self, config: BatteryConfig | None = None, initial_soc: float = 1.0
    ) -> None:
        self._config = config if config is not None else BatteryConfig()
        self._soc = min(1.0, max(0.0, initial_soc))
        self._current_a = 0.0

    @property
    def soc(self) -> float:
        """Current state of charge (0..1)."""
        return self._soc

    @property
    def is_low(self) -> bool:
        """True below 20% state of charge (RTL trigger threshold)."""
        return self._soc < 0.20

    @property
    def is_critical(self) -> bool:
        """True below 10% state of charge."""
        return self._soc < 0.10

    def force_soc(self, soc: float) -> None:
        """Set the state of charge directly (fault-injection hook).

        Clamped to [0, 1]. Used by the RTL scenario to inject a low
        battery without draining the pack organically.
        """
        self._soc = min(1.0, max(0.0, soc))

    def update(self, thrust_n: float, dt: float) -> BatteryState:
        """Advance one step under the given collective thrust (N).

        Current is proportional to thrust with an idle floor; charge is
        integrated (Coulomb counting) and voltage applies the IR drop.
        Returns the state AFTER the update.
        """
        if dt < 0.0:
            raise ValueError(f"dt must be non-negative, got {dt}")
        cfg = self._config
        load_current = cfg.hover_current_a * max(0.0, thrust_n) / cfg.hover_thrust_n
        self._current_a = max(cfg.idle_current_a, load_current)

        charge_ah = self._current_a * dt / 3600.0
        self._soc = max(0.0, self._soc - charge_ah / (cfg.capacity_mah / 1000.0))

        voltage = cfg.nominal_voltage * (0.9 + 0.1 * self._soc)
        voltage -= self._current_a * cfg.internal_resistance
        return BatteryState(soc=self._soc, voltage=voltage, current_a=self._current_a)


__all__ = ["BatteryConfig", "BatteryModel", "BatteryState"]
