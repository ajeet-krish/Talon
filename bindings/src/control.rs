//! Cascaded PD autopilot bindings: gains, command and controller.

use pyo3::prelude::*;

use talon_core::controller::{CascadedController, ControlCommand, ControlGains};

use crate::convert::{vec3_from, vec3_to};
use crate::types::{PyState, PyVehicleParams};

/// Cascaded PID autopilot gains.
#[pyclass(name = "ControlGains")]
pub struct PyControlGains {
    inner: ControlGains,
}

#[pymethods]
impl PyControlGains {
    #[new]
    fn new() -> Self {
        Self {
            inner: ControlGains::default(),
        }
    }

    /// Analytically derived Crazyflie-class starting gains.
    #[staticmethod]
    fn defaults(py: Python<'_>) -> PyResult<Py<Self>> {
        Py::new(
            py,
            Self {
                inner: ControlGains::default(),
            },
        )
    }

    /// Position P gains: (m/s^2) per m of position error.
    #[getter]
    fn kp_pos(&self) -> Vec<f64> {
        vec3_to(&self.inner.kp_pos)
    }

    #[setter]
    fn set_kp_pos(&mut self, value: &Bound<'_, PyAny>) -> PyResult<()> {
        self.inner.kp_pos = vec3_from(value, "kp_pos")?;
        Ok(())
    }

    /// Position I gains: (m/s^2) per m*s of position error.
    #[getter]
    fn ki_pos(&self) -> Vec<f64> {
        vec3_to(&self.inner.ki_pos)
    }

    #[setter]
    fn set_ki_pos(&mut self, value: &Bound<'_, PyAny>) -> PyResult<()> {
        self.inner.ki_pos = vec3_from(value, "ki_pos")?;
        Ok(())
    }

    /// Velocity D gains: (m/s^2) per m/s of velocity error.
    #[getter]
    fn kd_pos(&self) -> Vec<f64> {
        vec3_to(&self.inner.kd_pos)
    }

    #[setter]
    fn set_kd_pos(&mut self, value: &Bound<'_, PyAny>) -> PyResult<()> {
        self.inner.kd_pos = vec3_from(value, "kd_pos")?;
        Ok(())
    }

    /// Attitude P gains: (N*m) per rad of attitude error.
    #[getter]
    fn kp_att(&self) -> Vec<f64> {
        vec3_to(&self.inner.kp_att)
    }

    #[setter]
    fn set_kp_att(&mut self, value: &Bound<'_, PyAny>) -> PyResult<()> {
        self.inner.kp_att = vec3_from(value, "kp_att")?;
        Ok(())
    }

    /// Attitude I gains: (N*m) per rad*s. Exposed but unused.
    #[getter]
    fn ki_att(&self) -> Vec<f64> {
        vec3_to(&self.inner.ki_att)
    }

    #[setter]
    fn set_ki_att(&mut self, value: &Bound<'_, PyAny>) -> PyResult<()> {
        self.inner.ki_att = vec3_from(value, "ki_att")?;
        Ok(())
    }

    /// Rate D gains: (N*m) per rad/s of rate error.
    #[getter]
    fn kd_att(&self) -> Vec<f64> {
        vec3_to(&self.inner.kd_att)
    }

    #[setter]
    fn set_kd_att(&mut self, value: &Bound<'_, PyAny>) -> PyResult<()> {
        self.inner.kd_att = vec3_from(value, "kd_att")?;
        Ok(())
    }

    fn __repr__(&self) -> String {
        let p = self.inner.kp_pos;
        format!("ControlGains(kp_pos=[{}, {}, {}])", p[0], p[1], p[2])
    }
}

/// Output of one controller step: collective thrust (N, along `-body-z`) and
/// body-frame moment (N*m).
#[pyclass(name = "ControlCommand")]
pub struct PyControlCommand {
    pub(crate) inner: ControlCommand,
}

#[pymethods]
impl PyControlCommand {
    #[new]
    fn new() -> Self {
        Self {
            inner: ControlCommand::default(),
        }
    }

    /// Collective thrust (N, along `-body-z`).
    #[getter]
    fn collective_thrust(&self) -> f64 {
        self.inner.collective_thrust
    }

    #[setter]
    fn set_collective_thrust(&mut self, value: f64) {
        self.inner.collective_thrust = value;
    }

    /// Body-frame moment (N*m).
    #[getter]
    fn moment(&self) -> Vec<f64> {
        vec3_to(&self.inner.moment)
    }

    #[setter]
    fn set_moment(&mut self, value: &Bound<'_, PyAny>) -> PyResult<()> {
        self.inner.moment = vec3_from(value, "moment")?;
        Ok(())
    }

    fn __repr__(&self) -> String {
        format!(
            "ControlCommand(collective_thrust={})",
            self.inner.collective_thrust
        )
    }
}

/// Cascaded PD autopilot: position outer loop, attitude inner loop.
#[pyclass(name = "CascadedController")]
pub struct PyCascadedController {
    inner: CascadedController,
}

#[pymethods]
impl PyCascadedController {
    #[new]
    #[pyo3(signature = (gains, params=None))]
    fn new(gains: PyRef<'_, PyControlGains>, params: Option<PyRef<'_, PyVehicleParams>>) -> Self {
        let params = params.map(|p| p.inner).unwrap_or_default();
        Self {
            inner: CascadedController::new(gains.inner, params),
        }
    }

    /// One controller update.
    ///
    /// `pos_desired` and `vel_desired` are NED, `yaw_desired` in rad; `dt` is
    /// the control period (drives the position integrator with
    /// back-calculation anti-windup on thrust saturation).
    fn step(
        &mut self,
        py: Python<'_>,
        estimated_state: PyRef<'_, PyState>,
        pos_desired: &Bound<'_, PyAny>,
        vel_desired: &Bound<'_, PyAny>,
        yaw_desired: f64,
        dt: f64,
    ) -> PyResult<Py<PyControlCommand>> {
        let pos = vec3_from(pos_desired, "pos_desired")?;
        let vel = vec3_from(vel_desired, "vel_desired")?;
        let command = self
            .inner
            .step(&estimated_state.inner, &pos, &vel, yaw_desired, dt);
        Py::new(py, PyControlCommand { inner: command })
    }

    /// Clear internal state (integral, desired rates, telemetry).
    fn reset(&mut self) {
        self.inner.reset();
    }

    /// Desired body rate for the inner loop (rad/s); zero by default.
    fn set_omega_desired(&mut self, omega_desired: &Bound<'_, PyAny>) -> PyResult<()> {
        self.inner
            .set_omega_desired(vec3_from(omega_desired, "omega_desired")?);
        Ok(())
    }

    /// Last gravity-compensated thrust-accel command (NED, m/s^2).
    fn last_a_des(&self) -> Vec<f64> {
        vec3_to(&self.inner.last_a_des())
    }

    /// Last attitude error vector (dimensionless).
    fn last_attitude_error(&self) -> Vec<f64> {
        vec3_to(&self.inner.last_attitude_error())
    }

    /// Current position-error integrator state (NED, m*s).
    fn integral_position(&self) -> Vec<f64> {
        vec3_to(&self.inner.integral_position())
    }

    /// Thrust saturation limit (N): `4 * mass * gravity`.
    fn thrust_max(&self) -> f64 {
        self.inner.thrust_max()
    }
}
