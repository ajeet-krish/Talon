//! Python boundary for the NLGL line-of-sight guidance law.
//!
//! The core's `guidance` module ([`talon_core::guidance`]) is the single
//! implementation. This file only marshals: the law, its tuning and its
//! per-call diagnostics are the same numbers the autopilot flies in the loop.
//! Positions and velocities arrive duck-typed (numpy arrays or plain lists)
//! and the velocity command comes back as a plain `list[float]`.
//!
//! `NLGLGuidance.config()` hands back a snapshot of the active tuning rather
//! than a live reference, exactly like `Ekf.nominal()` (documented in
//! `docs/decisions.md`): writing a field on the returned `NLGLConfig` does not
//! retune the guidance object, while the pure-Python module shares one live
//! dataclass. The parity suite pins that difference as a registered
//! divergence (`guidance.config_is_a_snapshot`).

use pyo3::prelude::*;

use talon_core::guidance::{GuidanceDiagnostics, NlglConfig, NlglGuidance};

use crate::convert::arr3_from;
use crate::mission::PyWaypointMission;

/// NLGL tuning parameters, with the same defaults as the Python dataclass.
#[pyclass(name = "NLGLConfig")]
pub struct PyNLGLConfig {
    inner: NlglConfig,
}

#[pymethods]
impl PyNLGLConfig {
    #[new]
    #[pyo3(signature = (
        zeta = None,
        period_t = None,
        l1_min = None,
        nu1_clip_deg = None,
        nu_clip_deg = None,
        cruise_speed = None,
        kp_alt = None,
        kd_alt = None,
        climb_rate_max = None,
        min_ground_speed_for_nu2 = None,
    ))]
    #[allow(clippy::too_many_arguments)] // one optional argument per tuning field
    fn new(
        zeta: Option<f64>,
        period_t: Option<f64>,
        l1_min: Option<f64>,
        nu1_clip_deg: Option<f64>,
        nu_clip_deg: Option<f64>,
        cruise_speed: Option<f64>,
        kp_alt: Option<f64>,
        kd_alt: Option<f64>,
        climb_rate_max: Option<f64>,
        min_ground_speed_for_nu2: Option<f64>,
    ) -> Self {
        let defaults = NlglConfig::default();
        Self {
            inner: NlglConfig {
                zeta: zeta.unwrap_or(defaults.zeta),
                period_t: period_t.unwrap_or(defaults.period_t),
                l1_min: l1_min.unwrap_or(defaults.l1_min),
                nu1_clip_deg: nu1_clip_deg.unwrap_or(defaults.nu1_clip_deg),
                nu_clip_deg: nu_clip_deg.unwrap_or(defaults.nu_clip_deg),
                cruise_speed: cruise_speed.unwrap_or(defaults.cruise_speed),
                kp_alt: kp_alt.unwrap_or(defaults.kp_alt),
                kd_alt: kd_alt.unwrap_or(defaults.kd_alt),
                climb_rate_max: climb_rate_max.unwrap_or(defaults.climb_rate_max),
                min_ground_speed_for_nu2: min_ground_speed_for_nu2
                    .unwrap_or(defaults.min_ground_speed_for_nu2),
            },
        }
    }

    /// Look-ahead damping ratio.
    #[getter]
    fn zeta(&self) -> f64 {
        self.inner.zeta
    }

    #[setter]
    fn set_zeta(&mut self, value: f64) {
        self.inner.zeta = value;
    }

    /// Look-ahead period (s).
    #[getter]
    fn period_t(&self) -> f64 {
        self.inner.period_t
    }

    #[setter]
    fn set_period_t(&mut self, value: f64) {
        self.inner.period_t = value;
    }

    /// Minimum look-ahead distance (m).
    #[getter]
    fn l1_min(&self) -> f64 {
        self.inner.l1_min
    }

    #[setter]
    fn set_l1_min(&mut self, value: f64) {
        self.inner.l1_min = value;
    }

    /// Cross-track capture angle clip (deg).
    #[getter]
    fn nu1_clip_deg(&self) -> f64 {
        self.inner.nu1_clip_deg
    }

    #[setter]
    fn set_nu1_clip_deg(&mut self, value: f64) {
        self.inner.nu1_clip_deg = value;
    }

    /// Total steering angle clip (deg).
    #[getter]
    fn nu_clip_deg(&self) -> f64 {
        self.inner.nu_clip_deg
    }

    #[setter]
    fn set_nu_clip_deg(&mut self, value: f64) {
        self.inner.nu_clip_deg = value;
    }

    /// Commanded ground speed (m/s).
    #[getter]
    fn cruise_speed(&self) -> f64 {
        self.inner.cruise_speed
    }

    #[setter]
    fn set_cruise_speed(&mut self, value: f64) {
        self.inner.cruise_speed = value;
    }

    /// Altitude P gain (m/s per m).
    #[getter]
    fn kp_alt(&self) -> f64 {
        self.inner.kp_alt
    }

    #[setter]
    fn set_kp_alt(&mut self, value: f64) {
        self.inner.kp_alt = value;
    }

    /// Altitude D gain (dimensionless).
    #[getter]
    fn kd_alt(&self) -> f64 {
        self.inner.kd_alt
    }

    #[setter]
    fn set_kd_alt(&mut self, value: f64) {
        self.inner.kd_alt = value;
    }

    /// Climb / descent rate limit (m/s).
    #[getter]
    fn climb_rate_max(&self) -> f64 {
        self.inner.climb_rate_max
    }

    #[setter]
    fn set_climb_rate_max(&mut self, value: f64) {
        self.inner.climb_rate_max = value;
    }

    /// Ground speed below which Nu2 is forced to zero (m/s).
    #[getter]
    fn min_ground_speed_for_nu2(&self) -> f64 {
        self.inner.min_ground_speed_for_nu2
    }

    #[setter]
    fn set_min_ground_speed_for_nu2(&mut self, value: f64) {
        self.inner.min_ground_speed_for_nu2 = value;
    }
}

/// Per-call NLGL diagnostics (SI units, radians). Read-only.
#[pyclass(name = "GuidanceDiagnostics")]
pub struct PyGuidanceDiagnostics {
    inner: GuidanceDiagnostics,
}

#[pymethods]
impl PyGuidanceDiagnostics {
    /// Signed cross-track error (m, positive is left of the path).
    #[getter]
    fn e_ct(&self) -> f64 {
        self.inner.e_ct
    }

    /// Cross-track angle (rad), clipped at `nu1_clip`.
    #[getter]
    fn nu1(&self) -> f64 {
        self.inner.nu1
    }

    /// Ground-velocity course angle relative to the track (rad).
    #[getter]
    fn nu2(&self) -> f64 {
        self.inner.nu2
    }

    /// `nu1 + nu2`, clipped at `nu_clip` (rad).
    #[getter]
    fn nu(&self) -> f64 {
        self.inner.nu
    }

    /// Look-ahead distance used (m).
    #[getter]
    fn l1(&self) -> f64 {
        self.inner.l1
    }

    /// Commanded course (rad, wrapped to `[-pi, pi)`).
    #[getter]
    fn chi_d(&self) -> f64 {
        self.inner.chi_d
    }

    /// Whether behind-A recovery steering is active this call.
    #[getter]
    fn behind_a_recovery(&self) -> bool {
        self.inner.behind_a_recovery
    }

    /// Diagnostic lateral acceleration, `K_L1` form (m/s^2).
    #[getter]
    fn a_lat(&self) -> f64 {
        self.inner.a_lat
    }
}

/// Stateless NLGL line-of-sight guidance for one leg or a whole mission.
#[pyclass(name = "NLGLGuidance")]
pub struct PyNLGLGuidance {
    inner: NlglGuidance,
}

#[pymethods]
impl PyNLGLGuidance {
    #[new]
    #[pyo3(signature = (config = None))]
    fn new(config: Option<PyRef<'_, PyNLGLConfig>>) -> Self {
        let config = config.map(|c| c.inner).unwrap_or_default();
        Self {
            inner: NlglGuidance::new(config),
        }
    }

    /// The active configuration (a snapshot; see the module docstring).
    #[getter]
    fn config(&self, py: Python<'_>) -> PyResult<Py<PyNLGLConfig>> {
        Py::new(
            py,
            PyNLGLConfig {
                inner: *self.inner.config(),
            },
        )
    }

    /// NLGL velocity command for the leg `wp_a -> wp_b`.
    ///
    /// `position_ned` is the current NED position (m) and `velocity_ned` the
    /// EKF NED ground velocity (m/s). Returns `(v_des, diagnostics)`.
    #[pyo3(signature = (position_ned, velocity_ned, wp_a, wp_b))]
    fn compute_leg(
        &self,
        py: Python<'_>,
        position_ned: &Bound<'_, PyAny>,
        velocity_ned: &Bound<'_, PyAny>,
        wp_a: &Bound<'_, PyAny>,
        wp_b: &Bound<'_, PyAny>,
    ) -> PyResult<(Vec<f64>, Py<PyGuidanceDiagnostics>)> {
        let (v_des, diagnostics) = self.inner.compute_leg(
            arr3_from(position_ned, "position_ned")?,
            arr3_from(velocity_ned, "velocity_ned")?,
            arr3_from(wp_a, "wp_a")?,
            arr3_from(wp_b, "wp_b")?,
        );
        Ok((
            v_des.to_vec(),
            Py::new(py, PyGuidanceDiagnostics { inner: diagnostics })?,
        ))
    }

    /// NLGL velocity command toward the mission's active waypoint.
    ///
    /// The active leg is `mission.previous_point -> active waypoint`.
    #[pyo3(signature = (position_ned, velocity_ned, mission))]
    fn compute_mission(
        &self,
        py: Python<'_>,
        position_ned: &Bound<'_, PyAny>,
        velocity_ned: &Bound<'_, PyAny>,
        mission: PyRef<'_, PyWaypointMission>,
    ) -> PyResult<(Vec<f64>, Py<PyGuidanceDiagnostics>)> {
        let (v_des, diagnostics) = self.inner.compute_mission(
            arr3_from(position_ned, "position_ned")?,
            arr3_from(velocity_ned, "velocity_ned")?,
            &mission.inner,
        );
        Ok((
            v_des.to_vec(),
            Py::new(py, PyGuidanceDiagnostics { inner: diagnostics })?,
        ))
    }
}

/// Register the guidance boundary on the extension module.
pub fn register(module: &Bound<'_, PyModule>) -> PyResult<()> {
    module.add_class::<PyNLGLConfig>()?;
    module.add_class::<PyGuidanceDiagnostics>()?;
    module.add_class::<PyNLGLGuidance>()?;
    Ok(())
}
