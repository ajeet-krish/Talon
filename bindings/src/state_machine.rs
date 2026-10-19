//! Python boundary for the flight-phase state machine.
//!
//! The core's `state_machine` module ([`talon_core::state_machine`]) is the
//! single implementation: the transition table, the guards and the event log
//! are the same code the autopilot and the browser run. This file only
//! marshals.
//!
//! Telemetry arrives duck-typed: any object with `position_ned`,
//! `velocity_ned`, `battery_soc`, `ekf_converged`, `ekf_diverged`,
//! `gps_outage_duration`, `mission_waypoint_count`, `mission_complete`,
//! `motors_armed` and `home_position_ned`, plus the optional
//! `emergency_descent_active` and `motor_fault` flags (False when omitted).
//! One `talon.state_machine.Telemetry` object therefore drives both
//! machines. The optional flags default only when `getattr` reports
//! `AttributeError`; an attribute that exists but raises on access
//! propagates its exception rather than silently reading as False and
//! suppressing a guard.
//!
//! The `phase` getter and the `event_log()` rows report phases as the
//! lowercase label strings `FlightPhase.as_str()` produces (`"idle"`,
//! `"takeoff"`, ...), identical to the Python `FlightPhase.value` strings.
//! `force_phase` reads the same labels back, either from a
//! `FlightPhase.value` attribute or from a plain string.
//!
//! Documented divergences from the retired Python module, pinned by the
//! parity suite:
//!
//! * The constructor rejects a NaN `takeoff_altitude` (the Rust core checks
//!   `is_nan()` explicitly, because `NaN <= 0.0` is False), while the Python
//!   guard accepted it silently. The NaN message prints `NaN` (Rust float
//!   formatting) where Python would print `nan`, part of the same registered
//!   divergence (`state_machine.nan_takeoff_altitude`).
//! * The Python module type-checks (`isinstance`) and raises `TypeError` for
//!   a non-`Telemetry` in `request` or a non-`FlightPhase` in `force_phase`;
//!   this boundary duck-types instead, so the error surface differs
//!   (`state_machine.request_duck_types_telemetry`,
//!   `state_machine.force_phase_accepts_labels`).

use pyo3::exceptions::{PyAttributeError, PyValueError};
use pyo3::prelude::*;
use pyo3::types::{PyDict, PyList};

use talon_core::state_machine::{FlightPhaseMachine, Telemetry};
use talon_core::types::FlightPhase;

use crate::convert::arr3_from;

/// The message for a non-positive or NaN takeoff altitude, or `None` when it
/// is usable.
///
/// Kept pure so the message the Python tests match on is testable without an
/// initialized interpreter. The predicate mirrors `FlightPhaseMachine::new`
/// in `core/src/state_machine.rs`, which also rejects NaN explicitly. The
/// `{:?}` float formatting reproduces Python's `str()` text verbatim
/// (`0.0`, `-0.0`, `-1.0`).
fn takeoff_altitude_error(takeoff_altitude: f64) -> Option<String> {
    (takeoff_altitude <= 0.0 || takeoff_altitude.is_nan())
        .then(|| format!("takeoff_altitude must be positive, got {takeoff_altitude:?}"))
}

/// Read a flight phase from a `FlightPhase.value` attribute or a plain
/// lowercase label string.
fn phase_from(value: &Bound<'_, PyAny>) -> PyResult<FlightPhase> {
    let label: String = match value.getattr("value") {
        Ok(attr) => attr.extract()?,
        Err(_) => value.extract()?,
    };
    match label.as_str() {
        "idle" => Ok(FlightPhase::Idle),
        "arm" => Ok(FlightPhase::Arm),
        "takeoff" => Ok(FlightPhase::Takeoff),
        "cruise" => Ok(FlightPhase::Cruise),
        "waypoint" => Ok(FlightPhase::Waypoint),
        "rtl" => Ok(FlightPhase::Rtl),
        "land" => Ok(FlightPhase::Land),
        "emergency" => Ok(FlightPhase::Emergency),
        other => Err(PyValueError::new_err(format!(
            "unknown flight phase label {other:?}, expected one of the FlightPhase \
             values idle, arm, takeoff, cruise, waypoint, rtl, land, emergency"
        ))),
    }
}

/// A boolean guard flag the caller may omit entirely (False when absent).
///
/// Only a genuinely missing attribute defaults: `getattr` raising
/// `AttributeError` is Python's own signal for "no such attribute", so
/// that is the single case treated as absent. An attribute that exists
/// but raises on access (a broken property) propagates its exception
/// instead of silently reading as False, which could suppress a guard.
fn optional_bool(value: &Bound<'_, PyAny>, name: &str) -> PyResult<bool> {
    match value.getattr(name) {
        Ok(attr) => Ok(attr.extract()?),
        Err(error) if error.is_instance_of::<PyAttributeError>(value.py()) => Ok(false),
        Err(error) => Err(error),
    }
}

/// Read a telemetry snapshot from any object with the guard fields.
///
/// `emergency_descent_active` and `motor_fault` default to False when the
/// duck-typed object omits them, matching the Python dataclass defaults.
fn telemetry_from(value: &Bound<'_, PyAny>) -> PyResult<Telemetry> {
    Ok(Telemetry {
        position_ned: arr3_from(&value.getattr("position_ned")?, "position_ned")?,
        velocity_ned: arr3_from(&value.getattr("velocity_ned")?, "velocity_ned")?,
        battery_soc: value.getattr("battery_soc")?.extract()?,
        ekf_converged: value.getattr("ekf_converged")?.extract()?,
        ekf_diverged: value.getattr("ekf_diverged")?.extract()?,
        gps_outage_duration: value.getattr("gps_outage_duration")?.extract()?,
        mission_waypoint_count: value.getattr("mission_waypoint_count")?.extract()?,
        mission_complete: value.getattr("mission_complete")?.extract()?,
        motors_armed: value.getattr("motors_armed")?.extract()?,
        home_position_ned: arr3_from(&value.getattr("home_position_ned")?, "home_position_ned")?,
        emergency_descent_active: optional_bool(value, "emergency_descent_active")?,
        motor_fault: optional_bool(value, "motor_fault")?,
    })
}

/// Guarded eight-state flight phase machine with a transition event log.
#[pyclass(name = "FlightPhaseMachine")]
pub struct PyFlightPhaseMachine {
    inner: FlightPhaseMachine,
}

#[pymethods]
impl PyFlightPhaseMachine {
    #[new]
    #[pyo3(signature = (takeoff_altitude = 10.0))]
    fn new(takeoff_altitude: f64) -> PyResult<Self> {
        match takeoff_altitude_error(takeoff_altitude) {
            Some(message) => Err(PyValueError::new_err(message)),
            None => Ok(Self {
                inner: FlightPhaseMachine::new(takeoff_altitude)
                    .map_err(|error| PyValueError::new_err(error.to_string()))?,
            }),
        }
    }

    /// The current flight phase, as its lowercase label
    /// (matching the Python `FlightPhase.value` strings).
    #[getter]
    fn phase(&self) -> &'static str {
        self.inner.phase().as_str()
    }

    /// The takeoff altitude the `altitude_reached` guard uses (m).
    #[getter]
    fn takeoff_altitude(&self) -> f64 {
        self.inner.takeoff_altitude()
    }

    /// All event evaluations, oldest first, accepted and rejected alike.
    ///
    /// Each row is a read-only dict with keys `time`, `event`, `from_phase`,
    /// `to_phase` and `guard_ok`; the phase fields are lowercase labels.
    fn event_log(&self, py: Python<'_>) -> PyResult<Py<PyList>> {
        let rows = PyList::empty(py);
        for entry in self.inner.event_log() {
            let row = PyDict::new(py);
            row.set_item("time", entry.time)?;
            row.set_item("event", entry.event.as_str())?;
            row.set_item("from_phase", entry.from_phase.as_str())?;
            row.set_item("to_phase", entry.to_phase.as_str())?;
            row.set_item("guard_ok", entry.guard_ok)?;
            rows.append(row)?;
        }
        Ok(rows.unbind())
    }

    /// Evaluate one event against the current phase.
    ///
    /// `telemetry` is duck-typed (see the module docstring); the same
    /// `talon.state_machine.Telemetry` object drives both machines.
    #[pyo3(signature = (event, telemetry, time_s))]
    fn request(
        &mut self,
        event: &str,
        telemetry: &Bound<'_, PyAny>,
        time_s: f64,
    ) -> PyResult<bool> {
        let snapshot = telemetry_from(telemetry)?;
        Ok(self.inner.request(event, &snapshot, time_s))
    }

    /// Test and scenario hook: set the phase directly, logging the jump.
    ///
    /// `phase` is a `FlightPhase.value` or an equivalent lowercase label
    /// string. The event field of the log entry is `force:<reason>`.
    #[pyo3(signature = (phase, time_s, reason))]
    fn force_phase(&mut self, phase: &Bound<'_, PyAny>, time_s: f64, reason: &str) -> PyResult<()> {
        self.inner.force_phase(phase_from(phase)?, time_s, reason);
        Ok(())
    }
}

/// Register the state-machine boundary on the extension module.
pub fn register(module: &Bound<'_, PyModule>) -> PyResult<()> {
    module.add_class::<PyFlightPhaseMachine>()?;
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::takeoff_altitude_error;

    #[test]
    fn a_non_positive_takeoff_altitude_is_rejected_with_the_python_message() {
        assert!(takeoff_altitude_error(10.0).is_none());
        assert!(takeoff_altitude_error(0.5).is_none());
        let message = takeoff_altitude_error(0.0).expect("a zero altitude is an error");
        assert_eq!(message, "takeoff_altitude must be positive, got 0.0");
        let message = takeoff_altitude_error(-1.0).expect("a negative altitude is an error");
        assert_eq!(message, "takeoff_altitude must be positive, got -1.0");
        let message = takeoff_altitude_error(-0.0).expect("a negative-zero altitude is an error");
        assert_eq!(message, "takeoff_altitude must be positive, got -0.0");
    }

    #[test]
    fn a_nan_takeoff_altitude_is_rejected() {
        // Regression: `<= 0.0` alone would accept NaN. Rust prints `NaN`
        // where Python's `str()` would print `nan`; the parity suite pins
        // that message edge inside the registered NaN divergence.
        let message = takeoff_altitude_error(f64::NAN).expect("a NaN altitude is an error");
        assert!(message.contains("NaN"), "{message}");
    }
}
