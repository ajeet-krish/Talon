//! Python boundary for the waypoint mission manager.
//!
//! The core's `mission` module ([`talon_core::mission`]) owns the waypoint
//! list, the active-index bookkeeping and the acceptance radius logic. This
//! file only marshals, exactly like the rest of the boundary: acceptance,
//! chaining and the completion latch are the same code the autopilot flies.
//!
//! Waypoints arrive duck-typed, matching the avoidance boundary: any object
//! with `position` and (optionally) `speed`, `heading` and `radius`, with the
//! same defaults the Python `Waypoint` dataclass uses. They come back as
//! plain tuples `(position, speed, heading, radius)` for the caller to wrap.
//! The optional fields default only when `getattr` reports `AttributeError`;
//! an attribute that exists but raises on access propagates its exception
//! rather than silently reading as the default.
//!
//! One behaviour follows the Rust core rather than the retired Python
//! module: `update` with a non-finite position does not advance the mission
//! and does not raise, because `core/src/mission.rs` returns early to keep
//! the control loop panic-free. The Python module raised a `ValueError`
//! there instead; the parity suite pins that difference as a registered
//! divergence (`mission.update_non_finite_position`) rather than papering it
//! over in the binding.

use pyo3::exceptions::{PyAttributeError, PyValueError};
use pyo3::prelude::*;

use talon_core::mission::WaypointMission;
use talon_core::types::Waypoint;

use crate::convert::arr3_from;

/// A waypoint presented as `(position, speed, heading, radius)`.
type WaypointTuple = (Vec<f64>, f64, f64, f64);

/// Read a waypoint from any object with `position`, and the usual defaults.
fn waypoint_from(value: &Bound<'_, PyAny>) -> PyResult<Waypoint> {
    let position = arr3_from(&value.getattr("position")?, "position")?;
    let speed = optional_f64(value, "speed")?.unwrap_or(5.0);
    let heading = optional_f64(value, "heading")?.unwrap_or(0.0);
    let radius = optional_f64(value, "radius")?.unwrap_or(2.0);
    Ok(Waypoint::new(position, speed, heading, radius))
}

/// An attribute that the caller may omit entirely.
///
/// Only a genuinely missing attribute defaults: `getattr` raising
/// `AttributeError` is Python's own signal for "no such attribute", so
/// that is the single case treated as absent. An attribute that exists
/// but raises on access (a broken property) propagates its exception
/// instead of silently reading as the default.
fn optional_f64(value: &Bound<'_, PyAny>, name: &str) -> PyResult<Option<f64>> {
    match value.getattr(name) {
        Ok(attr) => Ok(Some(attr.extract()?)),
        Err(error) if error.is_instance_of::<PyAttributeError>(value.py()) => Ok(None),
        Err(error) => Err(error),
    }
}

/// Present a core waypoint as a tuple for the Python side.
fn waypoint_to_tuple(waypoint: &Waypoint) -> WaypointTuple {
    (
        waypoint.position.to_vec(),
        waypoint.speed,
        waypoint.heading,
        waypoint.radius,
    )
}

/// The message for an empty waypoint list, or `None` when the list is usable.
///
/// Kept pure so the message the Python tests match on is testable without an
/// initialized interpreter. The text mirrors `python/talon/mission.py`
/// verbatim, which is what makes the error parity testable.
fn no_waypoints_error(waypoints: &[Waypoint]) -> Option<String> {
    waypoints
        .is_empty()
        .then(|| "waypoints must be non-empty".to_string())
}

/// Sequential waypoint mission with an active index and acceptance radius.
#[pyclass(name = "WaypointMission")]
pub struct PyWaypointMission {
    pub(crate) inner: WaypointMission,
}

#[pymethods]
impl PyWaypointMission {
    #[new]
    fn new(waypoints: &Bound<'_, PyAny>) -> PyResult<Self> {
        let mut route: Vec<Waypoint> = Vec::new();
        for waypoint in waypoints.try_iter()? {
            route.push(waypoint_from(&waypoint?)?);
        }
        if let Some(message) = no_waypoints_error(&route) {
            return Err(PyValueError::new_err(message));
        }
        let inner = WaypointMission::new(route)
            .map_err(|error| PyValueError::new_err(error.to_string()))?;
        Ok(Self { inner })
    }

    /// Index of the waypoint currently being flown toward.
    #[getter]
    fn active_index(&self) -> usize {
        self.inner.active_index()
    }

    /// True once the vehicle has been within radius of the last waypoint.
    #[getter]
    fn complete(&self) -> bool {
        self.inner.complete()
    }

    /// The mission waypoints as `(position, speed, heading, radius)` tuples.
    fn waypoints(&self) -> Vec<WaypointTuple> {
        self.inner
            .waypoints()
            .iter()
            .map(waypoint_to_tuple)
            .collect()
    }

    /// The waypoint currently being flown toward, as a tuple.
    fn active_waypoint(&self) -> WaypointTuple {
        waypoint_to_tuple(self.inner.active_waypoint())
    }

    /// Record the mission start position (leg origin of the first leg).
    fn set_start(&mut self, position_ned: &Bound<'_, PyAny>) -> PyResult<()> {
        self.inner
            .set_start(arr3_from(position_ned, "position_ned")?);
        Ok(())
    }

    /// Origin of the active leg (NED, m): waypoint `i-1` once past the
    /// first waypoint, otherwise the recorded start, otherwise
    /// `current_position`.
    fn previous_point(&self, current_position: &Bound<'_, PyAny>) -> PyResult<Vec<f64>> {
        let previous = self
            .inner
            .previous_point(arr3_from(current_position, "current_position")?);
        Ok(previous.to_vec())
    }

    /// Advance the active waypoint if the vehicle is within its radius.
    ///
    /// Returns `True` once the mission is complete; `complete` stays latched
    /// afterwards.
    fn update(&mut self, position_ned: &Bound<'_, PyAny>) -> PyResult<bool> {
        Ok(self.inner.update(arr3_from(position_ned, "position_ned")?))
    }

    /// Horizontal distance still to fly (m).
    fn distance_remaining(&self, position_ned: &Bound<'_, PyAny>) -> PyResult<f64> {
        Ok(self
            .inner
            .distance_remaining(arr3_from(position_ned, "position_ned")?))
    }
}

/// Register the mission boundary on the extension module.
pub fn register(module: &Bound<'_, PyModule>) -> PyResult<()> {
    module.add_class::<PyWaypointMission>()?;
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::no_waypoints_error;
    use talon_core::types::Waypoint;

    #[test]
    fn an_empty_waypoint_list_is_rejected_with_the_python_message() {
        let message = no_waypoints_error(&[]).expect("an empty list is an error");
        assert_eq!(message, "waypoints must be non-empty");
    }

    #[test]
    fn a_populated_waypoint_list_is_accepted() {
        let route = vec![Waypoint::new([0.0, 0.0, -10.0], 5.0, 0.0, 2.0)];
        assert!(no_waypoints_error(&route).is_none());
    }
}
