//! Battery model: Coulomb counting with a thrust-proportional load.
//!
//! Ported from the previous Python source (`battery.py`):
//!
//! ```text
//! I       = max(idle_current, hover_current * thrust / hover_thrust)
//! dAh     = I * dt / 3600
//! soc    -= dAh / capacity_mah
//! voltage = nominal_voltage * (0.9 + 0.1 * soc) - I * internal_resistance
//! ```
//!
//! Documented model choices, unchanged:
//!
//! * Current scales linearly with collective thrust. Motor electrical power
//!   goes as torque times speed, both proportional to thrust for a fixed-pitch
//!   prop, so the linear map is the standard first-order SIL approximation. An
//!   idle floor keeps the pack draining slowly at zero thrust, like real
//!   avionics and idling ESCs.
//! * Voltage is a linear open-circuit-versus-soc droop (100 percent at full
//!   charge, 90 percent at empty) minus an ohmic drop under load. No hysteresis
//!   and no temperature: enough for RTL trigger logic, not for power design.
//! * 1500 mAh at about 5 A hover gives roughly 18 minutes of hover endurance.
//!   Scenarios inject a low battery with `force_soc` rather than draining
//!   organically.

use crate::constants::{GRAVITY, MASS};

/// Battery electrical and load parameters.
#[derive(Debug, Clone, Copy)]
pub struct BatteryConfig {
    /// Pack capacity, mAh.
    pub capacity_mah: f64,
    /// Nominal pack voltage, V.
    pub nominal_voltage: f64,
    /// Current drawn at hover thrust, A.
    pub hover_current_a: f64,
    /// Collective thrust at hover, N.
    pub hover_thrust_n: f64,
    /// Pack internal resistance, ohm.
    pub internal_resistance: f64,
    /// Avionics and idle ESC draw, A.
    pub idle_current_a: f64,
}

impl Default for BatteryConfig {
    fn default() -> Self {
        Self {
            capacity_mah: 1500.0,
            nominal_voltage: 11.1,
            hover_current_a: 5.0,
            // mg, matching the hover thrust of the default vehicle.
            hover_thrust_n: MASS * GRAVITY,
            internal_resistance: 0.05,
            idle_current_a: 0.2,
        }
    }
}

/// Why a battery model could not be constructed.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum BatteryError {
    /// Capacity must be strictly positive.
    NonPositiveCapacity(f64),
    /// Hover thrust must be strictly positive.
    NonPositiveHoverThrust(f64),
}

impl std::fmt::Display for BatteryError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::NonPositiveCapacity(v) => {
                write!(f, "capacity_mah must be positive, got {v}")
            }
            Self::NonPositiveHoverThrust(v) => {
                write!(f, "hover_thrust_n must be positive, got {v}")
            }
        }
    }
}

impl std::error::Error for BatteryError {}

impl BatteryConfig {
    /// Validate the configuration.
    pub fn validate(&self) -> Result<(), BatteryError> {
        if !(self.capacity_mah > 0.0) {
            return Err(BatteryError::NonPositiveCapacity(self.capacity_mah));
        }
        if !(self.hover_thrust_n > 0.0) {
            return Err(BatteryError::NonPositiveHoverThrust(self.hover_thrust_n));
        }
        Ok(())
    }
}

/// One battery sample: state of charge, voltage and current.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct BatteryState {
    /// State of charge, 0 to 1.
    pub soc: f64,
    /// Terminal voltage, V.
    pub voltage: f64,
    /// Load current, A.
    pub current_a: f64,
}

/// Coulomb-counting battery with a thrust-proportional load and IR drop.
#[derive(Debug, Clone)]
pub struct BatteryModel {
    config: BatteryConfig,
    soc: f64,
    current_a: f64,
}

impl BatteryModel {
    /// Build a battery with the given configuration and initial charge.
    pub fn new(config: BatteryConfig, initial_soc: f64) -> Result<Self, BatteryError> {
        config.validate()?;
        Ok(Self {
            config,
            soc: initial_soc.clamp(0.0, 1.0),
            current_a: 0.0,
        })
    }

    /// Build with the default pack, fully charged.
    pub fn with_defaults() -> Self {
        Self::new(BatteryConfig::default(), 1.0).expect("the default pack is valid")
    }

    /// Current state of charge, 0 to 1.
    pub fn soc(&self) -> f64 {
        self.soc
    }

    /// True below 20 percent, the RTL trigger threshold.
    pub fn is_low(&self) -> bool {
        self.soc < 0.20
    }

    /// True below 10 percent.
    pub fn is_critical(&self) -> bool {
        self.soc < 0.10
    }

    /// Current load, A.
    pub fn current_a(&self) -> f64 {
        self.current_a
    }

    /// Set the state of charge directly, clamped to `[0, 1]`.
    ///
    /// This is the fault-injection hook the RTL scenario uses to inject a low
    /// battery without draining the pack organically.
    pub fn force_soc(&mut self, soc: f64) {
        self.soc = soc.clamp(0.0, 1.0);
    }

    /// Advance one step under the given collective thrust (N).
    ///
    /// Current is proportional to thrust with an idle floor, charge is
    /// integrated by Coulomb counting, and voltage applies the IR drop.
    /// Returns the state after the update.
    pub fn update(&mut self, thrust_n: f64, dt: f64) -> BatteryState {
        let dt = if dt > 0.0 { dt } else { 0.0 };
        let cfg = &self.config;
        let load_current = cfg.hover_current_a * f64::max(0.0, thrust_n) / cfg.hover_thrust_n;
        self.current_a = f64::max(cfg.idle_current_a, load_current);

        let charge_ah = self.current_a * dt / 3600.0;
        self.soc = f64::max(0.0, self.soc - charge_ah / (cfg.capacity_mah / 1000.0));

        let mut voltage = cfg.nominal_voltage * (0.9 + 0.1 * self.soc);
        voltage -= self.current_a * cfg.internal_resistance;
        BatteryState {
            soc: self.soc,
            voltage,
            current_a: self.current_a,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_default_pack_starts_full() {
        let b = BatteryModel::with_defaults();
        assert_eq!(b.soc(), 1.0);
        assert!(!b.is_low());
        assert!(!b.is_critical());
    }

    #[test]
    fn an_invalid_config_is_rejected() {
        let bad_capacity = BatteryConfig {
            capacity_mah: 0.0,
            ..Default::default()
        };
        assert!(matches!(
            BatteryModel::new(bad_capacity, 1.0),
            Err(BatteryError::NonPositiveCapacity(_))
        ));
        let bad_hover = BatteryConfig {
            hover_thrust_n: 0.0,
            ..Default::default()
        };
        assert!(matches!(
            BatteryModel::new(bad_hover, 1.0),
            Err(BatteryError::NonPositiveHoverThrust(_))
        ));
    }

    #[test]
    fn initial_charge_is_clamped_to_the_unit_interval() {
        let over = BatteryModel::new(BatteryConfig::default(), 5.0).unwrap();
        assert_eq!(over.soc(), 1.0);
        let under = BatteryModel::new(BatteryConfig::default(), -3.0).unwrap();
        assert_eq!(under.soc(), 0.0);
    }

    #[test]
    fn hover_load_is_the_hover_current() {
        let mut b = BatteryModel::with_defaults();
        let cfg = BatteryConfig::default();
        let state = b.update(cfg.hover_thrust_n, 0.005);
        assert!((state.current_a - cfg.hover_current_a).abs() < 1e-12, "{state:?}");
    }

    #[test]
    fn current_scales_linearly_with_thrust() {
        let cfg = BatteryConfig::default();
        let mut b = BatteryModel::new(cfg, 1.0).unwrap();
        let state = b.update(2.0 * cfg.hover_thrust_n, 0.005);
        assert!((state.current_a - 2.0 * cfg.hover_current_a).abs() < 1e-12);
    }

    #[test]
    fn the_idle_floor_keeps_the_pack_draining_at_zero_thrust() {
        let cfg = BatteryConfig::default();
        let mut b = BatteryModel::new(cfg, 1.0).unwrap();
        let before = b.soc();
        let state = b.update(0.0, 60.0);
        assert!((state.current_a - cfg.idle_current_a).abs() < 1e-12, "{state:?}");
        assert!(b.soc() < before, "should still drain slowly");
    }

    #[test]
    fn hover_endurance_matches_the_documented_eighteen_minutes() {
        // 1500 mAh at 5 A is 0.3 h = 18 min. Clock it out step by step.
        let cfg = BatteryConfig::default();
        let mut b = BatteryModel::new(cfg, 1.0).unwrap();
        let mut t = 0.0;
        let dt = 0.1;
        while b.soc() > 0.0 && t < 3600.0 {
            b.update(cfg.hover_thrust_n, dt);
            t += dt;
        }
        let minutes = t / 60.0;
        assert!(
            (minutes - 18.0).abs() < 0.2,
            "hover endurance {minutes} min, expected about 18"
        );
    }

    #[test]
    fn voltage_droops_with_charge() {
        let cfg = BatteryConfig::default();
        let mut full = BatteryModel::new(cfg, 1.0).unwrap();
        let mut empty = BatteryModel::new(cfg, 0.0).unwrap();
        let v_full = full.update(0.0, 0.0).voltage;
        let v_empty = empty.update(0.0, 0.0).voltage;
        assert!(v_full > v_empty, "{v_full} vs {v_empty}");
    }

    #[test]
    fn voltage_droops_under_load() {
        let cfg = BatteryConfig::default();
        let mut light = BatteryModel::new(cfg, 0.5).unwrap();
        let mut heavy = BatteryModel::new(cfg, 0.5).unwrap();
        let v_light = light.update(0.0, 0.0).voltage;
        let v_heavy = heavy.update(4.0 * cfg.hover_thrust_n, 0.0).voltage;
        assert!(v_light > v_heavy, "{v_light} vs {v_heavy}");
    }

    #[test]
    fn the_low_thresholds_are_strict() {
        let mut b = BatteryModel::with_defaults();
        b.force_soc(0.20);
        assert!(!b.is_low(), "20 percent is not yet low");
        b.force_soc(0.199);
        assert!(b.is_low());
        b.force_soc(0.10);
        assert!(!b.is_critical(), "10 percent is not yet critical");
        b.force_soc(0.099);
        assert!(b.is_critical());
    }

    #[test]
    fn force_soc_clamps_and_the_pack_never_goes_negative() {
        let mut b = BatteryModel::with_defaults();
        b.force_soc(2.0);
        assert_eq!(b.soc(), 1.0);
        b.force_soc(-1.0);
        assert_eq!(b.soc(), 0.0);
        b.force_soc(0.01);
        for _ in 0..10_000 {
            b.update(20.0, 1.0);
        }
        assert_eq!(b.soc(), 0.0, "soc must floor at zero");
    }

    #[test]
    fn a_zero_length_step_drains_nothing() {
        let mut b = BatteryModel::with_defaults();
        let before = b.soc();
        b.update(20.0, 0.0);
        assert_eq!(b.soc(), before);
    }

    #[test]
    fn a_negative_step_is_treated_as_zero() {
        let mut b = BatteryModel::with_defaults();
        let before = b.soc();
        b.update(20.0, -5.0);
        assert_eq!(b.soc(), before);
    }

    #[test]
    fn negative_thrust_is_clamped_to_no_load() {
        let cfg = BatteryConfig::default();
        let mut b = BatteryModel::new(cfg, 1.0).unwrap();
        let state = b.update(-50.0, 0.005);
        assert!((state.current_a - cfg.idle_current_a).abs() < 1e-12, "{state:?}");
    }
}
