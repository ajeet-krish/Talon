//! Cascaded PID position and attitude autopilot.
//!
//! Ported term for term from the previous C++ core (`controller.hpp`), which
//! follows Beard and McLain chapter 6 for the outer/inner loop split and the Lee
//! et al. geometric attitude error, specialized to Talon's NED and
//! Hamilton-quaternion conventions.
//!
//! Outer loop (position PID, kinematic acceleration command):
//!
//! ```text
//! a_cmd = kp_pos .* (p_des - p) + kd_pos .* (v_des - v) + ki_pos .* I
//! ```
//!
//! Inner loop (attitude PD):
//!
//! ```text
//! M = kp_att .* e_att + kd_att .* (omega_des - omega)
//! ```
//!
//! with `a_des = [0, 0, g] - a_cmd`, `T = clamp(mass * ||a_des||, 0, T_MAX)` and
//! `z_b_des = a_des / ||a_des||`. The position integrator uses back-calculation
//! anti-windup (Astrom): while thrust saturates, the integral is scaled by
//! `lambda = T_MAX / T_unsat` so it forgets exactly the untrackable error rather
//! than clipping discontinuously.
//!
//! Sign notes, both forced by thrust acting along `-body-z`:
//!
//! 1. `a_des = g_ned - a_cmd`, not `g_ned + a_cmd`. A north position error must
//!    tilt the vehicle nose down so thrust tilts north.
//! 2. `e_att` is the negative of the Lee et al. `e_R = 0.5 vee(R_err - R_err^T)`.
//!
//! Phase 2 simplifications retained: no gyroscopic feedforward, no rate
//! feedforward (`omega_desired` defaults to zero), no attitude integral
//! (`ki_att` exists for API symmetry but is unused).

use crate::math;
use crate::types::{Mat3, State, Vec3, VehicleParams};

/// Vee operator: skew-symmetric matrix to 3-vector.
pub fn vee(s: &Mat3) -> Vec3 {
    Vec3::new(s[(2, 1)], s[(0, 2)], s[(1, 0)])
}

/// Cascaded PID autopilot gains.
#[derive(Debug, Clone, Copy)]
pub struct ControlGains {
    /// Position P gains: (m/s^2) per m of position error.
    pub kp_pos: Vec3,
    /// Position I gains: (m/s^2) per m*s of position error.
    pub ki_pos: Vec3,
    /// Velocity D gains: (m/s^2) per m/s of velocity error.
    pub kd_pos: Vec3,
    /// Attitude P gains: (N m) per rad of attitude error.
    pub kp_att: Vec3,
    /// Attitude I gains: (N m) per rad*s. Exposed but unused.
    pub ki_att: Vec3,
    /// Rate D gains: (N m) per rad/s of rate error.
    pub kd_att: Vec3,
}

impl Default for ControlGains {
    /// Analytically derived starting points, unchanged from the C++ core.
    ///
    /// Outer loop: `omega_n = 2 rad/s`, `zeta = 0.75` gives `kp = 4`, `kd = 3`.
    /// The integral gain sets `tau_i = kp/ki = 13.3 s`, slow enough not to eat
    /// the PD phase margin and fast enough to null steady-state wind drift.
    /// Inner loop roll and pitch: `omega_a ~ 58.7 rad/s`, `zeta ~ 1.1`. Yaw is
    /// capped by the physical yaw envelope (`c_tau = MOTOR_KM/MOTOR_KF = 0.015 m`
    /// gives at most `c_tau * m * g ~ 0.22 N m`), so `kp_att.z = 0.4` already
    /// saturates at a 30 degree error.
    fn default() -> Self {
        Self {
            kp_pos: Vec3::new(4.0, 4.0, 4.0),
            kd_pos: Vec3::new(3.0, 3.0, 3.0),
            ki_pos: Vec3::new(0.3, 0.3, 0.3),
            ki_att: Vec3::new(0.0, 0.0, 0.0),
            kp_att: Vec3::new(8.0, 8.0, 0.4),
            kd_att: Vec3::new(0.3, 0.3, 0.064),
        }
    }
}

/// Output of one controller step: collective thrust (N, along `-body-z`) and
/// body-frame moment (N m). Motor mixing is [`crate::motors`]' job.
#[derive(Debug, Clone, Copy, Default)]
pub struct ControlCommand {
    pub collective_thrust: f64,
    pub moment: Vec3,
}

/// Cascaded PID autopilot.
#[derive(Debug, Clone)]
pub struct CascadedController {
    gains: ControlGains,
    params: VehicleParams,
    /// Thrust saturation limit: `4 * mass * gravity`, a 4g thrust-to-weight cap.
    /// The raw motor limit (`4 * MOTOR_KF * M_MAX^2 ~ 2048 N`) is about 139x
    /// hover here and therefore non-binding.
    thrust_max: f64,
    omega_desired: Vec3,
    integral_pos: Vec3,
    last_a_des: Vec3,
    last_attitude_error: Vec3,
}

impl CascadedController {
    /// Build with explicit gains and vehicle parameters.
    pub fn new(gains: ControlGains, params: VehicleParams) -> Self {
        Self {
            thrust_max: 4.0 * params.mass * params.gravity,
            gains,
            params,
            omega_desired: Vec3::zeros(),
            integral_pos: Vec3::zeros(),
            last_a_des: Vec3::zeros(),
            last_attitude_error: Vec3::zeros(),
        }
    }

    /// Build with default gains and default vehicle parameters.
    pub fn with_defaults() -> Self {
        Self::new(ControlGains::default(), VehicleParams::default())
    }

    /// One control step. `estimated_state` is the EKF estimate; positions and
    /// velocities in NED (m, m/s) and `yaw_desired` in rad. `dt` is the control
    /// period; the position integrator accumulates `pos_error * dt`.
    pub fn step(
        &mut self,
        estimated_state: &State,
        pos_desired: &Vec3,
        vel_desired: &Vec3,
        yaw_desired: f64,
        dt: f64,
    ) -> ControlCommand {
        let pos_error = pos_desired - estimated_state.position;
        let vel_error = vel_desired - estimated_state.velocity;
        self.integral_pos += pos_error * dt;
        let a_cmd = self.gains.kp_pos.component_mul(&pos_error)
            + self.gains.kd_pos.component_mul(&vel_error)
            + self.gains.ki_pos.component_mul(&self.integral_pos);

        // Gravity-compensated thrust-acceleration command. Thrust acts along
        // -body-z, so the thrust acceleration is -a_des and z_b_des = a_des
        // normalized: at hover a_des = [0, 0, g] and the vehicle sits level.
        let a_des = Vec3::new(0.0, 0.0, self.params.gravity) - a_cmd;
        self.last_a_des = a_des;

        let a_norm = a_des.norm();
        let thrust_unsat = self.params.mass * a_norm;
        let thrust = math::clamp(thrust_unsat, 0.0, self.thrust_max);
        if thrust_unsat > self.thrust_max && a_norm > 1e-12 {
            let lambda = self.thrust_max / thrust_unsat;
            self.integral_pos *= lambda;
        }

        // Desired body-z, guarded against the free-fall degenerate case where
        // the position PD briefly commands exactly -g during vertical braking.
        let z_b = if a_norm > 1e-9 {
            a_des / a_norm
        } else {
            Vec3::new(0.0, 0.0, 1.0)
        };

        // Desired attitude from z_b and the yaw command: project the yaw
        // heading onto the plane orthogonal to z_b to get the other two axes.
        let x_c = Vec3::new(math::cos(yaw_desired), math::sin(yaw_desired), 0.0);
        let mut y_b = z_b.cross(&x_c);
        if y_b.norm() < 1e-6 {
            let fallback = if z_b[2].abs() < 0.9 {
                Vec3::new(0.0, 0.0, 1.0)
            } else {
                Vec3::new(1.0, 0.0, 0.0)
            };
            y_b = z_b.cross(&fallback);
        }
        let y_b = y_b / y_b.norm();
        let x_b = y_b.cross(&z_b);
        let r_des = Mat3::from_columns(&[x_b, y_b, z_b]);

        let r_actual = estimated_state.orientation.to_dcm();
        let r_err = r_des.transpose() * r_actual;
        self.last_attitude_error = 0.5 * vee(&(r_err.transpose() - r_err));

        let rate_error = self.omega_desired - estimated_state.angular_rate;

        ControlCommand {
            collective_thrust: thrust,
            moment: self.gains.kp_att.component_mul(&self.last_attitude_error)
                + self.gains.kd_att.component_mul(&rate_error),
        }
    }

    /// Clear all internal state.
    pub fn reset(&mut self) {
        self.omega_desired = Vec3::zeros();
        self.integral_pos = Vec3::zeros();
        self.last_a_des = Vec3::zeros();
        self.last_attitude_error = Vec3::zeros();
    }

    /// Desired body rate for the inner loop (rad/s, body frame).
    pub fn set_omega_desired(&mut self, omega_desired: Vec3) {
        self.omega_desired = omega_desired;
    }

    /// Gravity-compensated thrust-acceleration command (m/s^2, NED).
    pub fn last_a_des(&self) -> Vec3 {
        self.last_a_des
    }

    /// Attitude error vector (dimensionless, roughly radians for small errors).
    pub fn last_attitude_error(&self) -> Vec3 {
        self.last_attitude_error
    }

    /// Position-error integrator state (m s, NED).
    pub fn integral_position(&self) -> Vec3 {
        self.integral_pos
    }

    /// Thrust saturation limit (N).
    pub fn thrust_max(&self) -> f64 {
        self.thrust_max
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::dynamics::propagate;
    use crate::math::to_radians;
    use crate::types::Quaternion;

    #[test]
    fn level_hover_command_is_vertical_and_hover_thrust() {
        let mut c = CascadedController::with_defaults();
        let state = State::default();
        let cmd = c.step(&state, &Vec3::zeros(), &Vec3::zeros(), 0.0, 0.01);
        assert!((cmd.collective_thrust - 1.5 * 9.81).abs() < 1e-9, "{}", cmd.collective_thrust);
        assert!(cmd.moment.norm() < 1e-12, "moment {}", cmd.moment);
    }

    #[test]
    fn north_position_error_commands_northward_thrust() {
        // `a_des` is the thrust-acceleration reference, and thrust acts along
        // its negation (thrust is along -body-z, z_b_des = a_des normalized).
        // So a northward position error must make a_des point southward. The
        // sign is confirmed end to end by the closed-loop hover test.
        let mut c = CascadedController::with_defaults();
        let state = State::default();
        c.step(&state, &Vec3::new(5.0, 0.0, 0.0), &Vec3::zeros(), 0.0, 0.01);
        let a_des = c.last_a_des();
        assert!(a_des[0] < 0.0, "a_des should point south: {a_des}");
        // Thrust acceleration is -a_des, which points north here.
        assert!(-a_des[0] > 0.0);
        // And the vertical component stays near gravity.
        assert!((a_des[2] - 9.81).abs() < 1e-9);
    }

    #[test]
    fn yaw_command_turns_the_desired_attitude() {
        let mut c = CascadedController::with_defaults();
        let state = State::default();
        // Level, at the setpoint, but with a 90 degree yaw command: the
        // attitude error must be non-zero and about the yaw axis.
        let cmd = c.step(&state, &Vec3::zeros(), &Vec3::zeros(), to_radians(90.0), 0.01);
        let e = c.last_attitude_error();
        assert!(e[2].abs() > 1e-3, "expected yaw error: {e}");
        assert!(cmd.moment[2].abs() > 0.0, "expected yaw moment");
    }

    #[test]
    fn thrust_saturation_bounds_the_position_integrator() {
        // Back-calculation anti-windup scales the integral by
        // lambda = T_MAX / T_unsat while saturated, so it settles at a small
        // bounded value instead of accumulating. Without it, 200 steps of a
        // 1000 m error at dt = 0.01 s would reach 2000 m*s.
        let mut c = CascadedController::with_defaults();
        let state = State::default();
        for _ in 0..200 {
            c.step(&state, &Vec3::new(1000.0, 0.0, 0.0), &Vec3::zeros(), 0.0, 0.01);
        }
        let accumulated = c.integral_position()[0].abs();
        assert!(accumulated < 1.0, "integrator should stay bounded, got {accumulated}");
        assert!(accumulated > 0.0, "integrator should not be clipped to zero");
    }

    #[test]
    fn small_errors_accumulate_normally() {
        // Below saturation the integrator must integrate the error freely.
        let mut c = CascadedController::with_defaults();
        let state = State::default();
        for _ in 0..100 {
            c.step(&state, &Vec3::new(0.05, 0.0, 0.0), &Vec3::zeros(), 0.0, 0.01);
        }
        // 100 steps * 0.05 m * 0.01 s = 0.05 m*s.
        assert!((c.integral_position()[0] - 0.05).abs() < 1e-9, "{}", c.integral_position()[0]);
    }

    #[test]
    fn closed_loop_hover_is_stable() {
        // Fly the controller against the real plant from a small offset and
        // confirm it converges rather than diverging.
        let params = VehicleParams::default();
        let mut c = CascadedController::with_defaults();
        let mut s = State {
            position: Vec3::new(2.0, -1.0, -3.0),
            ..Default::default()
        };
        let target = Vec3::new(0.0, 0.0, 0.0);
        for _ in 0..2000 {
            let cmd = c.step(&s, &target, &Vec3::zeros(), 0.0, 0.005);
            s = propagate(&s, cmd.collective_thrust, &cmd.moment, &Vec3::zeros(), 0.005, &params);
        }
        assert!(s.position.norm() < 0.5, "did not converge: {}", s.position);
        assert!(s.velocity.norm() < 0.5, "still moving: {}", s.velocity);
    }

    #[test]
    fn reset_clears_the_integrator() {
        let mut c = CascadedController::with_defaults();
        let state = State::default();
        for _ in 0..100 {
            c.step(&state, &Vec3::new(1.0, 0.0, 0.0), &Vec3::zeros(), 0.0, 0.01);
        }
        assert!(c.integral_position().norm() > 0.0);
        c.reset();
        assert!(c.integral_position().norm() == 0.0);
        assert_eq!(c.last_a_des(), Vec3::zeros());
    }

    #[test]
    fn roll_command_produces_roll_moment() {
        let mut c = CascadedController::with_defaults();
        let half = to_radians(20.0) / 2.0;
        let state = State {
            orientation: Quaternion::new(math::cos(half), math::sin(half), 0.0, 0.0),
            ..Default::default()
        };
        let cmd = c.step(&state, &Vec3::zeros(), &Vec3::zeros(), 0.0, 0.01);
        // Vehicle rolled +20 deg; the level command must roll it back negative.
        assert!(cmd.moment[0] < 0.0, "moment {}", cmd.moment);
    }
}
