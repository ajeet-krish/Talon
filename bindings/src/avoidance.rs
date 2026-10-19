//! Python boundary for keep-out-zone geometry and the offline detour planner.
//!
//! The core's `avoidance` module ([`talon_core::avoidance`]) is the single
//! implementation. This file only marshals, exactly like the rest of the
//! boundary: the geometry, the planner, and the clearance report are the same
//! functions the reactive law flies in the loop.
//!
//! Objects arrive duck-typed rather than as a registered class. Obstacles are
//! read for `center_ned` and (optionally) `radius_m` and `id`; waypoints for
//! `position` and (optionally) `speed`, `heading`, and `radius`. That keeps
//! `talon.types` the owner of the Python dataclasses and stops the extension
//! from importing the package it is published into. Waypoints come back as
//! plain tuples `(position, speed, heading, radius)` for the caller to wrap.
//!
//! A negative clearance margin is a caller error. It is rejected here with a
//! `ValueError`, which is where the retired pure-Python module raised it too.

use pyo3::exceptions::PyValueError;
use pyo3::prelude::*;

use talon_core::avoidance;
use talon_core::types::{to_arr3, Arr3, Obstacle, Waypoint};

use crate::convert::vec3_from;

/// Read a NED position from any sequence of three numbers.
fn arr3_from(value: &Bound<'_, PyAny>, what: &str) -> PyResult<Arr3> {
    Ok(to_arr3(&vec3_from(value, what)?))
}

/// Read a keep-out zone from any object with `center_ned` and `radius_m`.
fn obstacle_from(value: &Bound<'_, PyAny>) -> PyResult<Obstacle> {
    let center = arr3_from(&value.getattr("center_ned")?, "center_ned")?;
    let radius_m: f64 = value.getattr("radius_m")?.extract()?;
    let id = value
        .getattr("id")
        .ok()
        .and_then(|id| id.extract::<String>().ok())
        .unwrap_or_default();
    Ok(Obstacle {
        id,
        center_ned: center,
        radius_m,
    })
}

/// Read a waypoint from any object with `position`, and the usual defaults.
fn waypoint_from(value: &Bound<'_, PyAny>) -> PyResult<Waypoint> {
    let position = arr3_from(&value.getattr("position")?, "position")?;
    let speed = optional_f64(value, "speed")?.unwrap_or(5.0);
    let heading = optional_f64(value, "heading")?.unwrap_or(0.0);
    let radius = optional_f64(value, "radius")?.unwrap_or(2.0);
    Ok(Waypoint::new(position, speed, heading, radius))
}

/// An attribute that the caller may omit entirely.
fn optional_f64(value: &Bound<'_, PyAny>, name: &str) -> PyResult<Option<f64>> {
    match value.getattr(name) {
        Ok(attr) => Ok(Some(attr.extract()?)),
        Err(_) => Ok(None),
    }
}

/// The message for a negative margin, or `None` when the margin is acceptable.
///
/// Kept pure so the message the Python tests match on is testable without an
/// initialized interpreter.
fn margin_error(clearance_margin: f64) -> Option<String> {
    (clearance_margin < 0.0)
        .then(|| format!("clearance_margin must be non-negative, got {clearance_margin}"))
}

/// Reject a negative margin the way the retired Python module did.
fn check_margin(clearance_margin: f64) -> PyResult<()> {
    match margin_error(clearance_margin) {
        Some(message) => Err(PyValueError::new_err(message)),
        None => Ok(()),
    }
}

/// Minimum horizontal distance from a zone centre to the leg `p0` to `p1` (m).
#[pyfunction]
fn leg_obstacle_distance(
    p0: &Bound<'_, PyAny>,
    p1: &Bound<'_, PyAny>,
    obstacle: &Bound<'_, PyAny>,
) -> PyResult<f64> {
    let zone = obstacle_from(obstacle)?;
    Ok(avoidance::leg_obstacle_distance(
        arr3_from(p0, "p0")?,
        arr3_from(p1, "p1")?,
        &zone,
    ))
}

/// True when the horizontal leg passes within `radius + clearance_margin` of
/// the zone centre.
#[pyfunction]
#[pyo3(signature = (p0, p1, obstacle, clearance_margin = 0.0))]
fn leg_intersects_obstacle(
    p0: &Bound<'_, PyAny>,
    p1: &Bound<'_, PyAny>,
    obstacle: &Bound<'_, PyAny>,
    clearance_margin: f64,
) -> PyResult<bool> {
    check_margin(clearance_margin)?;
    let zone = obstacle_from(obstacle)?;
    Ok(avoidance::leg_intersects_obstacle(
        arr3_from(p0, "p0")?,
        arr3_from(p1, "p1")?,
        &zone,
        clearance_margin,
    ))
}

/// One detour waypoint that routes the leg around the zone, at the leg's
/// altitude.
#[pyfunction]
fn compute_detour_point(
    p0: &Bound<'_, PyAny>,
    p1: &Bound<'_, PyAny>,
    obstacle: &Bound<'_, PyAny>,
    clearance_margin: f64,
) -> PyResult<Vec<f64>> {
    check_margin(clearance_margin)?;
    let zone = obstacle_from(obstacle)?;
    let detour = avoidance::compute_detour_point(
        arr3_from(p0, "p0")?,
        arr3_from(p1, "p1")?,
        &zone,
        clearance_margin,
    );
    Ok(detour.to_vec())
}

/// A waypoint list with one detour waypoint inserted per threatened leg, as
/// `(position, speed, heading, radius)` tuples.
#[pyfunction]
fn plan_avoidance_mission(
    waypoints: &Bound<'_, PyAny>,
    obstacles: &Bound<'_, PyAny>,
    clearance_margin: f64,
) -> PyResult<Vec<(Vec<f64>, f64, f64, f64)>> {
    check_margin(clearance_margin)?;
    let mut route: Vec<Waypoint> = Vec::new();
    for waypoint in waypoints.try_iter()? {
        route.push(waypoint_from(&waypoint?)?);
    }
    let mut zones: Vec<Obstacle> = Vec::new();
    for obstacle in obstacles.try_iter()? {
        zones.push(obstacle_from(&obstacle?)?);
    }
    Ok(
        avoidance::plan_avoidance_mission(&route, &zones, clearance_margin)
            .into_iter()
            .map(|w| (w.position.to_vec(), w.speed, w.heading, w.radius))
            .collect(),
    )
}

/// Minimum 3-D surface clearance (m) over a trajectory, or `inf` when there
/// are no positions or no zones.
#[pyfunction]
fn min_clearance(positions: &Bound<'_, PyAny>, obstacles: &Bound<'_, PyAny>) -> PyResult<f64> {
    let mut samples: Vec<Arr3> = Vec::new();
    for position in positions.try_iter()? {
        samples.push(arr3_from(&position?, "position")?);
    }
    let mut zones: Vec<Obstacle> = Vec::new();
    for obstacle in obstacles.try_iter()? {
        zones.push(obstacle_from(&obstacle?)?);
    }
    Ok(avoidance::min_surface_clearance(&samples, &zones))
}

/// Register the avoidance boundary on the extension module.
pub fn register(module: &Bound<'_, PyModule>) -> PyResult<()> {
    module.add_function(wrap_pyfunction!(leg_obstacle_distance, module)?)?;
    module.add_function(wrap_pyfunction!(leg_intersects_obstacle, module)?)?;
    module.add_function(wrap_pyfunction!(compute_detour_point, module)?)?;
    module.add_function(wrap_pyfunction!(plan_avoidance_mission, module)?)?;
    module.add_function(wrap_pyfunction!(min_clearance, module)?)?;
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::margin_error;

    #[test]
    fn a_negative_margin_is_rejected() {
        assert!(margin_error(0.0).is_none());
        assert!(margin_error(2.0).is_none());
        let message = margin_error(-1.0).expect("a negative margin is an error");
        assert!(message.contains("non-negative"), "{message}");
        assert!(message.contains("-1"), "{message}");
    }
}
