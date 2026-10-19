//! Rigid-body dynamics: 13-state RK4 quadrotor in NED.
//!
//! Ported term for term from the previous C++ core (`dynamics.hpp`). The state
//! is `[position(3), quaternion(4), velocity(3), angular_rate(3)]`, integrated
//! with classical RK4 and the quaternion renormalized at the end of each step.
//!
//! Thrust convention: collective thrust `T >= 0` acts along `-body-z`, so a level
//! vehicle is pushed in `-NED-z` (upward). Drag is quadratic in the air-relative
//! velocity. Angular acceleration is the Newton-Euler gyroscopic form.

use crate::types::{Mat3, Quaternion, State, Vec3, VehicleParams};

/// Hamilton quaternion product `a * b`.
pub fn quat_multiply(a: &Quaternion, b: &Quaternion) -> Quaternion {
    Quaternion::new(
        a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z,
        a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
        a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
        a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w,
    )
}

/// Renormalized quaternion.
pub fn quat_normalize(q: &Quaternion) -> Quaternion {
    q.normalized()
}

/// Quaternion derivative for body-frame angular rate:
/// `q_dot = 0.5 * q * [0, omega_body]`.
pub fn quat_derivative(q: &Quaternion, omega_body: &Vec3) -> Quaternion {
    let omega_q = Quaternion::new(0.0, omega_body[0], omega_body[1], omega_body[2]);
    let qd = quat_multiply(q, &omega_q);
    Quaternion::new(0.5 * qd.w, 0.5 * qd.x, 0.5 * qd.y, 0.5 * qd.z)
}

/// Body-to-NED direction cosine matrix from a Hamilton quaternion.
pub fn rotation_matrix(q: &Quaternion) -> Mat3 {
    q.to_dcm()
}

/// Componentwise `r = s + a * d`, quaternion treated as a 4-vector.
///
/// Used only for intermediate RK4 stages; the final quaternion is renormalized.
pub fn state_add_scaled(s: &State, d: &State, a: f64) -> State {
    State {
        position: s.position + a * d.position,
        velocity: s.velocity + a * d.velocity,
        angular_rate: s.angular_rate + a * d.angular_rate,
        orientation: Quaternion::new(
            s.orientation.w + a * d.orientation.w,
            s.orientation.x + a * d.orientation.x,
            s.orientation.y + a * d.orientation.y,
            s.orientation.z + a * d.orientation.z,
        ),
    }
}

/// Continuous-time Newton-Euler derivative in NED.
pub fn state_derivative(
    s: &State,
    thrust: f64,
    moment: &Vec3,
    wind_ned: &Vec3,
    p: &VehicleParams,
) -> State {
    let mut d = State {
        position: s.velocity,
        ..Default::default()
    };

    let r = rotation_matrix(&s.orientation);
    let thrust_body = Vec3::new(0.0, 0.0, -thrust);
    let v_rel = s.velocity - wind_ned;
    let k_drag = 0.5 * p.air_density * p.drag_coeff * p.vehicle_area;
    d.velocity = (r * thrust_body) / p.mass + Vec3::new(0.0, 0.0, p.gravity)
        - (k_drag * v_rel.norm() * v_rel) / p.mass;

    d.orientation = quat_derivative(&s.orientation, &s.angular_rate);

    let inertia = p.inertia_diag;
    let i_omega = inertia.component_mul(&s.angular_rate);
    let gyro = s.angular_rate.cross(&i_omega);
    d.angular_rate = (moment - gyro).component_div(&inertia);
    d
}

/// Integrate one RK4 step, renormalizing the quaternion.
pub fn propagate(
    state: &State,
    thrust: f64,
    moment: &Vec3,
    wind_ned: &Vec3,
    dt: f64,
    params: &VehicleParams,
) -> State {
    let k1 = state_derivative(state, thrust, moment, wind_ned, params);
    let k2 = state_derivative(
        &state_add_scaled(state, &k1, 0.5 * dt),
        thrust,
        moment,
        wind_ned,
        params,
    );
    let k3 = state_derivative(
        &state_add_scaled(state, &k2, 0.5 * dt),
        thrust,
        moment,
        wind_ned,
        params,
    );
    let k4 = state_derivative(
        &state_add_scaled(state, &k3, dt),
        thrust,
        moment,
        wind_ned,
        params,
    );

    let sixth = dt / 6.0;
    let out = State {
        position: state.position
            + sixth * (k1.position + 2.0 * k2.position + 2.0 * k3.position + k4.position),
        velocity: state.velocity
            + sixth * (k1.velocity + 2.0 * k2.velocity + 2.0 * k3.velocity + k4.velocity),
        angular_rate: state.angular_rate
            + sixth
                * (k1.angular_rate
                    + 2.0 * k2.angular_rate
                    + 2.0 * k3.angular_rate
                    + k4.angular_rate),
        orientation: Quaternion::new(
            state.orientation.w
                + sixth
                    * (k1.orientation.w
                        + 2.0 * k2.orientation.w
                        + 2.0 * k3.orientation.w
                        + k4.orientation.w),
            state.orientation.x
                + sixth
                    * (k1.orientation.x
                        + 2.0 * k2.orientation.x
                        + 2.0 * k3.orientation.x
                        + k4.orientation.x),
            state.orientation.y
                + sixth
                    * (k1.orientation.y
                        + 2.0 * k2.orientation.y
                        + 2.0 * k3.orientation.y
                        + k4.orientation.y),
            state.orientation.z
                + sixth
                    * (k1.orientation.z
                        + 2.0 * k2.orientation.z
                        + 2.0 * k3.orientation.z
                        + k4.orientation.z),
        ),
    };
    State {
        orientation: out.orientation.normalized(),
        ..out
    }
}

/// Thrust needed to hover (level, no acceleration).
pub fn hover_thrust_with(params: &VehicleParams) -> f64 {
    params.mass * params.gravity
}

/// Hover thrust for the default vehicle.
pub fn hover_thrust() -> f64 {
    hover_thrust_with(&VehicleParams::default())
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::math::to_radians;

    fn params() -> VehicleParams {
        VehicleParams::default()
    }

    #[test]
    fn zero_thrust_is_free_fall() {
        // Level vehicle, no thrust, no drag (zero velocity keeps drag zero at
        // the start, and we take one small step).
        let s = State::default();
        let out = propagate(&s, 0.0, &Vec3::zeros(), &Vec3::zeros(), 0.005, &params());
        // After 5 ms of free fall: v = g*t downward, z = 0.5*g*t^2 downward.
        assert!(
            (out.velocity[2] - 9.81 * 0.005).abs() < 1e-6,
            "{}",
            out.velocity[2]
        );
        assert!((out.position[2] - 0.5 * 9.81 * 0.005 * 0.005).abs() < 1e-9);
    }

    #[test]
    fn hover_thrust_holds_altitude() {
        let p = params();
        let s = State::default();
        let t = hover_thrust_with(&p);
        let mut state = s;
        for _ in 0..200 {
            state = propagate(&state, t, &Vec3::zeros(), &Vec3::zeros(), 0.005, &p);
        }
        assert!(
            state.position[2].abs() < 1e-3,
            "drifted {}",
            state.position[2]
        );
        assert!(
            state.velocity[2].abs() < 1e-3,
            "climbing {}",
            state.velocity[2]
        );
    }

    #[test]
    fn quaternion_stays_unit_under_integration() {
        let p = params();
        let mut state = State::default();
        let moment = Vec3::new(0.01, -0.005, 0.002);
        for _ in 0..1000 {
            state = propagate(
                &state,
                hover_thrust_with(&p),
                &moment,
                &Vec3::zeros(),
                0.005,
                &p,
            );
        }
        assert!((state.orientation.norm() - 1.0).abs() < 1e-12);
    }

    #[test]
    fn drag_opposes_motion() {
        // Moving north in still air, drag must decelerate northward motion.
        let p = params();
        let s = State {
            velocity: Vec3::new(10.0, 0.0, 0.0),
            ..State::default()
        };
        let d = state_derivative(
            &s,
            hover_thrust_with(&p),
            &Vec3::zeros(),
            &Vec3::zeros(),
            &p,
        );
        assert!(d.velocity[0] < 0.0, "drag should oppose northward motion");
    }

    #[test]
    fn momentum_from_moment_signs() {
        // Positive roll moment on a level vehicle should increase roll rate.
        let p = params();
        let s = State::default();
        let d = state_derivative(
            &s,
            hover_thrust_with(&p),
            &Vec3::new(0.05, 0.0, 0.0),
            &Vec3::zeros(),
            &p,
        );
        assert!(d.angular_rate[0] > 0.0);
    }

    #[test]
    fn wind_creates_relative_airflow() {
        // Hovering in an 8 m/s wind blowing toward the south (NED -north): the
        // air-relative velocity is +north, so drag must push the vehicle
        // southward, which is downwind. Ground velocity is zero yet drag appears.
        let p = params();
        let s = State::default();
        let wind = Vec3::new(-8.0, 0.0, 0.0);
        let d = state_derivative(&s, hover_thrust_with(&p), &Vec3::zeros(), &wind, &p);
        assert!(
            d.velocity[0] < 0.0,
            "wind should carry the vehicle downwind, got {}",
            d.velocity[0]
        );
    }

    #[test]
    fn yaw_rate_produces_heading_change() {
        let p = params();
        let mut s = State {
            angular_rate: Vec3::new(0.0, 0.0, to_radians(90.0)),
            ..State::default()
        };
        for _ in 0..200 {
            s = propagate(
                &s,
                hover_thrust_with(&p),
                &Vec3::zeros(),
                &Vec3::zeros(),
                0.005,
                &p,
            );
        }
        // 1 s at 90 deg/s about down is a quarter turn; body +x now points east.
        let v = s.orientation.rotate_vector(&Vec3::new(1.0, 0.0, 0.0));
        assert!(v[1] > 0.9, "expected eastward heading, got {v}");
    }
}
