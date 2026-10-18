//! Quaternion, state, vehicle-parameter and wind bindings, plus the free
//! dynamics functions the harness calls directly.

use pyo3::prelude::*;

use talon_core::dynamics;
use talon_core::types::{Quaternion, State, VehicleParams};
use talon_core::wind::DrydenWind;

use crate::convert::{mat3_to, vec3_from, vec3_to};

/// Hamilton quaternion, scalar first `[w, x, y, z]`.
#[pyclass(name = "Quaternion")]
pub struct PyQuaternion {
    /// Scalar part.
    #[pyo3(get, set)]
    pub w: f64,
    /// Vector x part.
    #[pyo3(get, set)]
    pub x: f64,
    /// Vector y part.
    #[pyo3(get, set)]
    pub y: f64,
    /// Vector z part.
    #[pyo3(get, set)]
    pub z: f64,
}

impl PyQuaternion {
    pub(crate) fn to_core(&self) -> Quaternion {
        Quaternion::new(self.w, self.x, self.y, self.z)
    }

    pub(crate) fn from_core(q: Quaternion) -> Self {
        Self {
            w: q.w,
            x: q.x,
            y: q.y,
            z: q.z,
        }
    }
}

#[pymethods]
impl PyQuaternion {
    #[new]
    #[pyo3(signature = (w=1.0, x=0.0, y=0.0, z=0.0))]
    fn new(w: f64, x: f64, y: f64, z: f64) -> Self {
        Self { w, x, y, z }
    }

    /// Euclidean norm of the four components.
    fn norm(&self) -> f64 {
        self.to_core().norm()
    }

    /// Unit-norm copy.
    fn normalized(&self, py: Python<'_>) -> PyResult<Py<Self>> {
        Py::new(py, Self::from_core(self.to_core().normalized()))
    }

    /// Inverse rotation (unit quaternion assumption).
    fn conjugate(&self, py: Python<'_>) -> PyResult<Py<Self>> {
        Py::new(py, Self::from_core(self.to_core().conjugate()))
    }

    /// Rotate a vector from body frame to NED frame.
    fn rotate_vector(&self, v: &Bound<'_, PyAny>) -> PyResult<Vec<f64>> {
        let body = vec3_from(v, "v")?;
        Ok(vec3_to(&self.to_core().rotate_vector(&body)))
    }

    /// Body-to-NED direction cosine matrix (3x3, row major).
    fn to_dcm(&self) -> Vec<Vec<f64>> {
        mat3_to(&self.to_core().to_dcm())
    }

    fn __repr__(&self) -> String {
        format!(
            "Quaternion(w={}, x={}, y={}, z={})",
            self.w, self.x, self.y, self.z
        )
    }
}

/// Full vehicle state: NED position and velocity, body rate, orientation.
#[pyclass(name = "State")]
pub struct PyState {
    pub(crate) inner: State,
}

#[pymethods]
impl PyState {
    #[new]
    fn new() -> Self {
        Self {
            inner: State::default(),
        }
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

    /// Body angular rate (rad/s).
    #[getter]
    fn angular_rate(&self) -> Vec<f64> {
        vec3_to(&self.inner.angular_rate)
    }

    #[setter]
    fn set_angular_rate(&mut self, value: &Bound<'_, PyAny>) -> PyResult<()> {
        self.inner.angular_rate = vec3_from(value, "angular_rate")?;
        Ok(())
    }

    /// Body-to-NED quaternion.
    #[getter]
    fn orientation(&self, py: Python<'_>) -> PyResult<Py<PyQuaternion>> {
        Py::new(py, PyQuaternion::from_core(self.inner.orientation))
    }

    #[setter]
    fn set_orientation(&mut self, value: PyRef<'_, PyQuaternion>) -> PyResult<()> {
        self.inner.orientation = value.to_core();
        Ok(())
    }

    fn __repr__(&self) -> String {
        let p = self.inner.position;
        format!("State(position=[{}, {}, {}])", p[0], p[1], p[2])
    }
}

/// Vehicle physical parameters, defaulting to [`talon_core::constants`].
#[pyclass(name = "VehicleParams")]
pub struct PyVehicleParams {
    pub(crate) inner: VehicleParams,
}

#[pymethods]
impl PyVehicleParams {
    #[new]
    fn new() -> Self {
        Self {
            inner: VehicleParams::default(),
        }
    }

    /// Mass (kg).
    #[getter]
    fn mass(&self) -> f64 {
        self.inner.mass
    }

    #[setter]
    fn set_mass(&mut self, value: f64) {
        self.inner.mass = value;
    }

    /// Diagonal inertia `[Ixx, Iyy, Izz]` (kg*m^2).
    #[getter]
    fn inertia_diag(&self) -> Vec<f64> {
        vec3_to(&self.inner.inertia_diag)
    }

    #[setter]
    fn set_inertia_diag(&mut self, value: &Bound<'_, PyAny>) -> PyResult<()> {
        self.inner.inertia_diag = vec3_from(value, "inertia_diag")?;
        Ok(())
    }

    /// Rotor arm length (m).
    #[getter]
    fn arm_length(&self) -> f64 {
        self.inner.arm_length
    }

    #[setter]
    fn set_arm_length(&mut self, value: f64) {
        self.inner.arm_length = value;
    }

    /// Parasitic drag coefficient (dimensionless).
    #[getter]
    fn drag_coeff(&self) -> f64 {
        self.inner.drag_coeff
    }

    #[setter]
    fn set_drag_coeff(&mut self, value: f64) {
        self.inner.drag_coeff = value;
    }

    /// Gravitational acceleration (m/s^2).
    #[getter]
    fn gravity(&self) -> f64 {
        self.inner.gravity
    }

    #[setter]
    fn set_gravity(&mut self, value: f64) {
        self.inner.gravity = value;
    }

    /// Air density (kg/m^3).
    #[getter]
    fn air_density(&self) -> f64 {
        self.inner.air_density
    }

    #[setter]
    fn set_air_density(&mut self, value: f64) {
        self.inner.air_density = value;
    }

    /// Reference area (m^2).
    #[getter]
    fn vehicle_area(&self) -> f64 {
        self.inner.vehicle_area
    }

    #[setter]
    fn set_vehicle_area(&mut self, value: f64) {
        self.inner.vehicle_area = value;
    }
}

/// Dryden turbulence generator (per-axis first-order Gauss-Markov).
#[pyclass(name = "DrydenWind")]
pub struct PyDrydenWind {
    inner: DrydenWind,
}

#[pymethods]
impl PyDrydenWind {
    #[new]
    #[pyo3(signature = (
        sigma_u = 2.0,
        sigma_v = 2.0,
        sigma_w = 1.0,
        l_u = 200.0,
        l_v = 200.0,
        l_w = 50.0,
        dt = talon_core::constants::DT,
        seed = 42,
    ))]
    fn new(
        sigma_u: f64,
        sigma_v: f64,
        sigma_w: f64,
        l_u: f64,
        l_v: f64,
        l_w: f64,
        dt: f64,
        seed: u64,
    ) -> Self {
        Self {
            inner: DrydenWind::new(
                [sigma_u, sigma_v, sigma_w],
                [l_u, l_v, l_w],
                dt,
                seed,
            ),
        }
    }

    /// Advance one step; returns the current wind `[u, v, w]` in NED (m/s).
    fn step(&mut self, airspeed: f64) -> Vec<f64> {
        vec3_to(&self.inner.step(airspeed))
    }

    /// Re-seed the generator and zero the wind state.
    fn reset(&mut self) {
        self.inner.reset();
    }

    /// Current wind state in NED (m/s).
    fn wind(&self) -> Vec<f64> {
        vec3_to(&self.inner.wind())
    }
}

/// One RK4 step of the 13-state rigid-body model.
///
/// Thrust acts along `-body-z`; `wind` is the NED wind velocity; the
/// quaternion is renormalized.
#[pyfunction]
#[pyo3(signature = (state, thrust, moment, wind, dt, params=None))]
pub fn propagate(
    py: Python<'_>,
    state: PyRef<'_, PyState>,
    thrust: f64,
    moment: &Bound<'_, PyAny>,
    wind: &Bound<'_, PyAny>,
    dt: f64,
    params: Option<PyRef<'_, PyVehicleParams>>,
) -> PyResult<Py<PyState>> {
    let moment = vec3_from(moment, "moment")?;
    let wind = vec3_from(wind, "wind")?;
    let params = params.map(|p| p.inner).unwrap_or_default();
    let out = dynamics::propagate(&state.inner, thrust, &moment, &wind, dt, &params);
    Py::new(py, PyState { inner: out })
}

/// Body-to-NED DCM from a Hamilton quaternion (3x3, row major).
#[pyfunction]
pub fn rotation_matrix(q: PyRef<'_, PyQuaternion>) -> Vec<Vec<f64>> {
    mat3_to(&dynamics::rotation_matrix(&q.to_core()))
}

/// Collective thrust (N) required to hover.
#[pyfunction]
#[pyo3(signature = (params=None))]
pub fn hover_thrust(params: Option<PyRef<'_, PyVehicleParams>>) -> f64 {
    match params {
        Some(p) => dynamics::hover_thrust_with(&p.inner),
        None => dynamics::hover_thrust(),
    }
}
