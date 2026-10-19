//! MEKF state-estimator bindings: config, nominal state and the filter.

use pyo3::prelude::*;

use talon_core::ekf::{Ekf, EkfConfig, EkfNominalState};

use crate::convert::{mat15_to, vec3_from, vec3_to, vec6_to};
use crate::types::PyQuaternion;

/// Noise densities, initial covariances and the innovation gate.
#[derive(Default)]
#[pyclass(name = "EKFConfig")]
pub struct PyEkfConfig {
    inner: EkfConfig,
}

#[pymethods]
impl PyEkfConfig {
    #[new]
    fn new() -> Self {
        Self::default()
    }

    /// Continuous accel noise density (m/s^2/sqrt(Hz)).
    #[getter]
    fn accel_noise_density(&self) -> f64 {
        self.inner.accel_noise_density
    }

    #[setter]
    fn set_accel_noise_density(&mut self, value: f64) {
        self.inner.accel_noise_density = value;
    }

    /// Continuous gyro noise density (rad/s/sqrt(Hz)).
    #[getter]
    fn gyro_noise_density(&self) -> f64 {
        self.inner.gyro_noise_density
    }

    #[setter]
    fn set_gyro_noise_density(&mut self, value: f64) {
        self.inner.gyro_noise_density = value;
    }

    /// Accel bias random walk (m/s^2/sqrt(s)).
    #[getter]
    fn accel_bias_walk(&self) -> f64 {
        self.inner.accel_bias_walk
    }

    #[setter]
    fn set_accel_bias_walk(&mut self, value: f64) {
        self.inner.accel_bias_walk = value;
    }

    /// Gyro bias random walk (rad/s/sqrt(s)).
    #[getter]
    fn gyro_bias_walk(&self) -> f64 {
        self.inner.gyro_bias_walk
    }

    #[setter]
    fn set_gyro_bias_walk(&mut self, value: f64) {
        self.inner.gyro_bias_walk = value;
    }

    /// GPS horizontal position 1-sigma (m).
    #[getter]
    fn gps_pos_noise(&self) -> f64 {
        self.inner.gps_pos_noise
    }

    #[setter]
    fn set_gps_pos_noise(&mut self, value: f64) {
        self.inner.gps_pos_noise = value;
    }

    /// GPS vertical position 1-sigma (m).
    #[getter]
    fn gps_pos_noise_vertical(&self) -> f64 {
        self.inner.gps_pos_noise_vertical
    }

    #[setter]
    fn set_gps_pos_noise_vertical(&mut self, value: f64) {
        self.inner.gps_pos_noise_vertical = value;
    }

    /// GPS velocity 1-sigma (m/s).
    #[getter]
    fn gps_vel_noise(&self) -> f64 {
        self.inner.gps_vel_noise
    }

    #[setter]
    fn set_gps_vel_noise(&mut self, value: f64) {
        self.inner.gps_vel_noise = value;
    }

    /// Chi-square innovation gate (6 DOF).
    #[getter]
    fn innovation_gate_chi2(&self) -> f64 {
        self.inner.innovation_gate_chi2
    }

    #[setter]
    fn set_innovation_gate_chi2(&mut self, value: f64) {
        self.inner.innovation_gate_chi2 = value;
    }

    /// Initial position standard deviation (m).
    #[getter]
    fn init_pos_std(&self) -> f64 {
        self.inner.init_pos_std
    }

    #[setter]
    fn set_init_pos_std(&mut self, value: f64) {
        self.inner.init_pos_std = value;
    }

    /// Initial velocity standard deviation (m/s).
    #[getter]
    fn init_vel_std(&self) -> f64 {
        self.inner.init_vel_std
    }

    #[setter]
    fn set_init_vel_std(&mut self, value: f64) {
        self.inner.init_vel_std = value;
    }

    /// Initial attitude standard deviation (rad).
    #[getter]
    fn init_att_std(&self) -> f64 {
        self.inner.init_att_std
    }

    #[setter]
    fn set_init_att_std(&mut self, value: f64) {
        self.inner.init_att_std = value;
    }

    /// Initial gyro bias standard deviation (rad/s).
    #[getter]
    fn init_gyro_bias_std(&self) -> f64 {
        self.inner.init_gyro_bias_std
    }

    #[setter]
    fn set_init_gyro_bias_std(&mut self, value: f64) {
        self.inner.init_gyro_bias_std = value;
    }

    /// Initial accel bias standard deviation (m/s^2).
    #[getter]
    fn init_accel_bias_std(&self) -> f64 {
        self.inner.init_accel_bias_std
    }

    #[setter]
    fn set_init_accel_bias_std(&mut self, value: f64) {
        self.inner.init_accel_bias_std = value;
    }
}

/// Nominal (mean) EKF state.
#[derive(Default)]
#[pyclass(name = "EKFNominalState")]
pub struct PyEkfNominalState {
    pub(crate) inner: EkfNominalState,
}

#[pymethods]
impl PyEkfNominalState {
    #[new]
    fn new() -> Self {
        Self::default()
    }

    /// NED position `[n, e, d]` (m).
    #[getter]
    fn position(&self) -> Vec<f64> {
        vec3_to(&self.inner.position)
    }

    #[setter]
    fn set_position(&mut self, value: &Bound<'_, PyAny>) -> PyResult<()> {
        self.inner.position = vec3_from(value, "position")?;
        Ok(())
    }

    /// NED velocity (m/s).
    #[getter]
    fn velocity(&self) -> Vec<f64> {
        vec3_to(&self.inner.velocity)
    }

    #[setter]
    fn set_velocity(&mut self, value: &Bound<'_, PyAny>) -> PyResult<()> {
        self.inner.velocity = vec3_from(value, "velocity")?;
        Ok(())
    }

    /// Body-to-NED Hamilton quaternion.
    #[getter]
    fn attitude(&self, py: Python<'_>) -> PyResult<Py<PyQuaternion>> {
        Py::new(py, PyQuaternion::from_core(self.inner.attitude))
    }

    #[setter]
    fn set_attitude(&mut self, value: PyRef<'_, PyQuaternion>) -> PyResult<()> {
        self.inner.attitude = value.to_core();
        Ok(())
    }

    /// Gyro bias (rad/s, body).
    #[getter]
    fn gyro_bias(&self) -> Vec<f64> {
        vec3_to(&self.inner.gyro_bias)
    }

    #[setter]
    fn set_gyro_bias(&mut self, value: &Bound<'_, PyAny>) -> PyResult<()> {
        self.inner.gyro_bias = vec3_from(value, "gyro_bias")?;
        Ok(())
    }

    /// Accel bias (m/s^2, body).
    #[getter]
    fn accel_bias(&self) -> Vec<f64> {
        vec3_to(&self.inner.accel_bias)
    }

    #[setter]
    fn set_accel_bias(&mut self, value: &Bound<'_, PyAny>) -> PyResult<()> {
        self.inner.accel_bias = vec3_from(value, "accel_bias")?;
        Ok(())
    }

    fn __repr__(&self) -> String {
        let p = self.inner.position;
        format!("EKFNominalState(position=[{}, {}, {}])", p[0], p[1], p[2])
    }
}

/// 15-state multiplicative error-state Kalman filter (MEKF).
///
/// Error-state ordering `[dp(3), dv(3), dtheta(3), dbg(3), dba(3)]`, body-frame
/// (right) attitude error: `q_true = q_est (x) dq`.
#[pyclass(name = "Ekf")]
pub struct PyEkf {
    inner: Ekf,
}

#[pymethods]
impl PyEkf {
    #[new]
    #[pyo3(signature = (config=None))]
    fn new(config: Option<PyRef<'_, PyEkfConfig>>) -> Self {
        let config = config.map(|c| c.inner).unwrap_or_default();
        Self {
            inner: Ekf::new(config),
        }
    }

    /// Set the nominal state and P to the configured initial covariances.
    fn initialize(&mut self, initial: PyRef<'_, PyEkfNominalState>) {
        self.inner.initialize(initial.inner);
    }

    /// Initialize level (roll = pitch = 0, yaw as given) from the assumed
    /// stationary specific force `[0, 0, -g]`.
    #[pyo3(signature = (position, velocity, yaw=0.0))]
    fn initialize_level(
        &mut self,
        position: &Bound<'_, PyAny>,
        velocity: &Bound<'_, PyAny>,
        yaw: f64,
    ) -> PyResult<()> {
        let position = vec3_from(position, "position")?;
        let velocity = vec3_from(velocity, "velocity")?;
        self.inner.initialize_level(position, velocity, yaw);
        Ok(())
    }

    /// Initialize, leveling roll/pitch from a stationary body-frame specific
    /// force measurement.
    #[pyo3(signature = (position, velocity, accel_body, yaw=0.0))]
    fn initialize_from_accel(
        &mut self,
        position: &Bound<'_, PyAny>,
        velocity: &Bound<'_, PyAny>,
        accel_body: &Bound<'_, PyAny>,
        yaw: f64,
    ) -> PyResult<()> {
        let position = vec3_from(position, "position")?;
        let velocity = vec3_from(velocity, "velocity")?;
        let accel_body = vec3_from(accel_body, "accel_body")?;
        self.inner
            .initialize_from_accel(position, velocity, accel_body, yaw);
        Ok(())
    }

    /// IMU propagation (every `DT_IMU`).
    ///
    /// `accel_meas` is the body specific force (hover reads `[0, 0, -g]`);
    /// `gyro_meas` the body rate.
    fn predict(
        &mut self,
        accel_meas: &Bound<'_, PyAny>,
        gyro_meas: &Bound<'_, PyAny>,
        dt: f64,
    ) -> PyResult<()> {
        let accel = vec3_from(accel_meas, "accel_meas")?;
        let gyro = vec3_from(gyro_meas, "gyro_meas")?;
        self.inner.predict(&accel, &gyro, dt);
        Ok(())
    }

    /// GPS position/velocity update in NED (every `DT_GPS`).
    ///
    /// Returns `True` if accepted, `False` if gated out (state unchanged).
    fn update(
        &mut self,
        gps_position: &Bound<'_, PyAny>,
        gps_velocity: &Bound<'_, PyAny>,
    ) -> PyResult<bool> {
        let position = vec3_from(gps_position, "gps_position")?;
        let velocity = vec3_from(gps_velocity, "gps_velocity")?;
        Ok(self.inner.update(&position, &velocity))
    }

    /// Nominal EKF state. A snapshot: mutate the returned object and hand it
    /// back with `set_nominal` to write it into the filter.
    fn nominal(&self, py: Python<'_>) -> PyResult<Py<PyEkfNominalState>> {
        Py::new(
            py,
            PyEkfNominalState {
                inner: *self.inner.nominal(),
            },
        )
    }

    /// Write a nominal state back into the filter (fault-injection hook).
    fn set_nominal(&mut self, state: PyRef<'_, PyEkfNominalState>) {
        *self.inner.mutable_nominal() = state.inner;
    }

    /// 15x15 covariance matrix as a list of row lists (a copy).
    fn covariance(&self) -> Vec<Vec<f64>> {
        mat15_to(&self.inner.covariance())
    }

    /// Diagonal position covariance `[Pnn, Pee, Pdd]` (m^2).
    ///
    /// A cheap projection of [`Self::covariance`] for the convergence checks
    /// that run every IMU step.
    fn position_variance(&self) -> Vec<f64> {
        let p = self.inner.covariance();
        vec![p[(0, 0)], p[(1, 1)], p[(2, 2)]]
    }

    /// Trace of the position covariance block `Pnn + Pee + Pdd` (m^2).
    fn position_covariance_trace(&self) -> f64 {
        let p = self.inner.covariance();
        p[(0, 0)] + p[(1, 1)] + p[(2, 2)]
    }

    /// Last GPS innovation `[dp(3), dv(3)]` (6-vector).
    fn last_innovation(&self) -> Vec<f64> {
        vec6_to(&self.inner.last_innovation())
    }

    /// Chi-square of the last GPS innovation.
    fn last_innovation_chi2(&self) -> f64 {
        self.inner.last_innovation_chi2()
    }

    /// Whether the last GPS update passed the innovation gate.
    fn last_update_accepted(&self) -> bool {
        self.inner.last_update_accepted()
    }

    /// Bias-corrected body rates `gyro_meas - gyro_bias`.
    fn estimated_body_rates(&self, gyro_meas: &Bound<'_, PyAny>) -> PyResult<Vec<f64>> {
        let gyro = vec3_from(gyro_meas, "gyro_meas")?;
        Ok(vec3_to(&self.inner.estimated_body_rates(&gyro)))
    }

    fn __repr__(&self) -> String {
        let p = self.inner.nominal().position;
        format!("Ekf(position=[{}, {}, {}])", p[0], p[1], p[2])
    }
}
