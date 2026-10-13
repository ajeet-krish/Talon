#pragma once

#include <algorithm>
#include <cmath>

#include "talon/types.hpp"

namespace talon {

// vee operator: skew-symmetric matrix -> 3-vector. Inverse of the cross-product
// matrix [w]_x = [[0, -wz, wy], [wz, 0, -wx], [-wy, wx, 0]].
inline Vec3 vee(const Mat3& s) {
    return Vec3(s(2, 1), s(0, 2), s(1, 0));
}

// Cascaded PID autopilot gains (Beard & McLain Ch. 6 cascaded-loop design,
// adapted to the Talon NED convention of docs/conventions.md).
//
// Outer loop (position PID, kinematic acceleration command):
//   a_cmd = kp_pos .* (p_des - p) + kd_pos .* (v_des - v) + ki_pos .* I
//   I     = integral of (p_des - p) dt, back-calculation anti-windup
// is a third-order loop; the proportional-derivative part sets the
// natural frequency omega_n and damping zeta:
//   kp_pos = omega_n^2,  kd_pos = 2 * zeta * omega_n.
// Defaults use omega_n = 2 rad/s (~0.3 Hz bandwidth) and zeta = 0.75, which
// keeps the outer loop ~25x slower than the inner attitude loop (classic
// time-scale separation).
//
// Inner loop (attitude PD):
//   M = kp_att .* e_att + kd_att .* (omega_des - omega)
// closes I * phi_ddot + kd_att * phi_dot + kp_att * phi = 0 per axis, so
//   kp_att = I * omega_a^2,  kd_att = 2 * zeta_a * I * omega_a.
// Defaults: roll/pitch omega_a ~ 58.7 rad/s (~9.3 Hz, zeta_a ~ 1.1) for
// IXX = IYY = 2.32e-3; yaw softer at omega_a ~ 10 rad/s (~1.6 Hz,
// zeta_a ~ 0.8) for IZZ = 4.05e-3. The yaw axis is set by the physical yaw
// authority, not by a time-scale argument: with the realistic drag arm
// c_tau = MOTOR_KM / MOTOR_KF = 0.015 m, the hover-thrust yaw envelope is
// only c_tau * m * g ~ 0.22 N*m, so kp_att.z = 0.4 already commands
// ~0.21 N*m at a 30 deg error (saturation-limited). The yaw rise-time
// spec is therefore relaxed relative to roll/pitch (which stay < 0.3 s);
// these are analytically derived starting points and the closed-loop
// hover and step-response tests define the acceptance envelope.
struct ControlGains {
    Vec3 kp_pos;  // position P gains: (m/s^2) per m of position error
    Vec3 ki_pos;  // position I gains: (m/s^2) per m*s of position error
    Vec3 kd_pos;  // velocity D gains: (m/s^2) per m/s of velocity error
    Vec3 kp_att;  // attitude P gains: (N*m) per rad of attitude error
    Vec3 ki_att;  // attitude I gains: (N*m) per rad*s of attitude error
    Vec3 kd_att;  // rate D gains: (N*m) per rad/s of rate error

    static ControlGains defaults();
};

// Output of one controller step: collective thrust (N, along -body-z) and
// body-frame moment (N*m). No motor mixing here; MotorModel owns that.
struct ControlCommand {
    double collective_thrust = 0.0;  // N, >= 0
    Vec3 moment = Vec3::Zero();      // N*m, body frame
};

// Cascaded PID position/attitude autopilot (Beard & McLain Ch. 6 outer/inner
// loop split; attitude error from Lee et al. geometric tracking control),
// specialized to the Talon NED + Hamilton-quaternion conventions:
//
//   a_cmd_ned = kp_pos .* (p_des - p) + kd_pos .* (v_des - v)
//             + ki_pos .* integral(p_des - p, t) dt
//   a_des     = [0, 0, g] - a_cmd_ned        (gravity-compensated thrust accel)
//   T         = clamp(mass * ||a_des||, 0, T_MAX)
//   z_b_des   = a_des / ||a_des||            (level at hover: [0, 0, 1])
//   R_des     = attitude from z_b_des and yaw_desired (Lee et al. construction)
//   e_att     = 0.5 * vee(R_err^T - R_err), R_err = R_des^T R_actual
//   M         = kp_att .* e_att + kd_att .* (omega_des - omega)
//
// The position integrator uses back-calculation anti-windup (Astrom): while
// thrust is saturated, the integral is scaled by lambda = T_MAX / T_unsat so
// the integrator "forgets" exactly the portion of the error that the plant
// could not track. This bounds the unwind transient after a saturation event
// (e.g. a large position step or a strong gust) without clipping the
// integrator discontinuously. ki_att exists in ControlGains for API symmetry
// but is unused until attitude integral action lands (Sprint 2.2).
//
// Sign notes (both forced by closed-loop stability under the Talon thrust
// convention T along -body-z):
// 1. a_des = g_ned - a_cmd_ned, not g_ned + a_cmd_ned. With thrust along
//    -body-z, the thrust-acceleration vector must be -a_des, so z_b_des must
//    point along g_ned - a_cmd_ned: a north position error tilts the vehicle
//    nose down (thrust vector tilts north) and it accelerates north. The
//    g_ned + a_cmd_ned form accelerates opposite the position error.
// 2. e_att is the negative of the Lee et al. e_R = 0.5 vee(R_err - R_err^T):
//    with e_att = -e_R, M = kp_att * e_att achieves the same closed-loop sign
//    as Lee's M = -k_R * e_R: positive e_att (vehicle needs to roll toward
//    desired) produces positive Mx.
//
// Phase 2 simplifications: no omega x (I*omega) gyroscopic feedforward, no
// rate feedforward (omega_desired defaults to zero; set_omega_desired() is
// the hook for Phase 4+ guidance feedforward), no attitude integral (ki_att
// is exposed but unused). The position integrator consumes dt each step.
class CascadedController {
public:
    CascadedController(const ControlGains& gains, const VehicleParams& params);

    // estimated_state: EKF output in later phases; truth in Phase 2 tests.
    // pos_desired, vel_desired in NED (m, m/s); yaw_desired in rad.
    // dt is the caller's control period; the position integrator accumulates
    // pos_error * dt with back-calculation anti-windup on thrust saturation.
    ControlCommand step(const State& estimated_state,
                        const Vec3& pos_desired,
                        const Vec3& vel_desired,
                        double yaw_desired, double dt);

    // Clear all internal state (integral, omega_desired, last-command
    // telemetry).
    void reset();

    // Desired body rate for the inner loop (rad/s, body frame). Defaults to
    // zero (Phase 2: pure damping).
    void set_omega_desired(const Vec3& omega_desired);

    // Telemetry from the most recent step(): gravity-compensated
    // thrust-acceleration command in NED (m/s^2) and attitude error vector
    // (dimensionless, ~radians for small errors). integral_position() is the
    // current position-error integrator state (m*s, NED), scaled by the
    // anti-windup back-calculation while thrust is saturated.
    const Vec3& last_a_des() const { return last_a_des_; }
    const Vec3& last_attitude_error() const { return last_attitude_error_; }
    const Vec3& integral_position() const { return integral_pos_; }

    // Thrust saturation limit (N): 4 * mass * gravity, i.e. a 4g
    // thrust-to-weight cap. The raw physical motor limit
    // 4 * MOTOR_KF * M_MAX^2 (~2048 N with the current constants) is ~139x
    // hover for this vehicle and thus non-binding; 4 * mass * gravity bounds
    // runaway position commands while leaving full authority for takeoff,
    // climbs, and braking.
    double thrust_max() const { return thrust_max_; }

private:
    ControlGains gains_;
    VehicleParams params_;
    double thrust_max_;
    Vec3 omega_desired_;
    Vec3 integral_pos_;
    Vec3 last_a_des_;
    Vec3 last_attitude_error_;
};

inline ControlGains ControlGains::defaults() {
    ControlGains g;
    // Outer loop: omega_n = 2 rad/s, zeta = 0.75 -> kp = 4, kd = 3.
    // Integral gain sets tau_i = kp/ki = 13.3 s: slow enough not to eat the
    // PD loop's phase margin, fast enough to null steady-state wind drift.
    g.kp_pos = Vec3(4.0, 4.0, 4.0);
    g.kd_pos = Vec3(3.0, 3.0, 3.0);
    g.ki_pos = Vec3(0.3, 0.3, 0.3);  // tau_i = kp/ki = 13.3 s
    g.ki_att = Vec3(0.0, 0.0, 0.0);
    // Inner loop: roll/pitch (I = 2.32e-3) omega_a ~ 58.7 rad/s, zeta ~ 1.1.
    // Yaw (I = 4.05e-3) is capped by the physical yaw envelope: with the
    // realistic drag arm c_tau = MOTOR_KM / MOTOR_KF = 0.015 m, a pure-yaw
    // hover maneuver can deliver at most c_tau * m * g ~ 0.22 N*m, so
    // kp_att.z = 0.4 (omega_a ~ 9.9 rad/s, zeta ~ 0.8) already commands
    // ~0.21 N*m at a 30 deg error and saturates beyond that; larger gains
    // only deepen saturation without adding real authority.
    g.kp_att = Vec3(8.0, 8.0, 0.4);
    g.kd_att = Vec3(0.3, 0.3, 0.064);
    return g;
}

inline CascadedController::CascadedController(const ControlGains& gains,
                                              const VehicleParams& params)
    : gains_(gains),
      params_(params),
      thrust_max_(4.0 * params.mass * params.gravity),
      omega_desired_(Vec3::Zero()),
      integral_pos_(Vec3::Zero()),
      last_a_des_(Vec3::Zero()),
      last_attitude_error_(Vec3::Zero()) {}

inline ControlCommand CascadedController::step(const State& estimated_state,
                                               const Vec3& pos_desired,
                                               const Vec3& vel_desired,
                                               double yaw_desired,
                                               double dt) {
    // Outer loop: kinematic acceleration command in NED, with position
    // integral action for zero steady-state error under constant
    // disturbances (e.g. wind drag bias).
    const Vec3 pos_error = pos_desired - estimated_state.position;
    const Vec3 vel_error = vel_desired - estimated_state.velocity;
    integral_pos_ += pos_error * dt;
    const Vec3 a_cmd = gains_.kp_pos.cwiseProduct(pos_error) +
                       gains_.kd_pos.cwiseProduct(vel_error) +
                       gains_.ki_pos.cwiseProduct(integral_pos_);

    // Gravity-compensated thrust-acceleration command. Thrust acts along
    // -body-z, so the thrust acceleration is -a_des and z_b_des = a_des
    // normalized: at hover a_des = [0, 0, g] and the vehicle sits level.
    const Vec3 a_des = Vec3(0.0, 0.0, params_.gravity) - a_cmd;
    last_a_des_ = a_des;

    const double a_norm = a_des.norm();
    const double thrust_unsat = params_.mass * a_norm;
    double thrust = std::clamp(thrust_unsat, 0.0, thrust_max_);
    // Back-calculation anti-windup (Astrom): scale integral by saturation
    // ratio so the integrator forgets the untrackable portion of the error.
    if (thrust_unsat > thrust_max_ && a_norm > 1e-12) {
        const double lambda = thrust_max_ / thrust_unsat;
        integral_pos_ *= lambda;
    }

    // Desired body-z, guarded against the free-fall degenerate case a_des ~ 0
    // (position PD can briefly command exactly -g in vertical braking).
    Vec3 z_b(0.0, 0.0, 1.0);
    if (a_norm > 1e-9) {
        z_b = a_des / a_norm;
    }

    // Desired attitude from z_b and the yaw command (Lee et al.): project the
    // yaw heading onto the plane orthogonal to z_b to get the other two axes.
    const Vec3 x_c(std::cos(yaw_desired), std::sin(yaw_desired), 0.0);
    Vec3 y_b = z_b.cross(x_c);
    if (y_b.norm() < 1e-6) {
        // body-z nearly parallel to the yaw reference: any orthogonal works.
        const Vec3 fallback = (std::abs(z_b.z()) < 0.9) ? Vec3(0.0, 0.0, 1.0)
                                                        : Vec3(1.0, 0.0, 0.0);
        y_b = z_b.cross(fallback);
    }
    y_b.normalize();
    const Vec3 x_b = y_b.cross(z_b);
    Mat3 R_des;
    R_des.col(0) = x_b;
    R_des.col(1) = y_b;
    R_des.col(2) = z_b;

    // Attitude error (Lee et al., with the leading minus folded in; see the
    // class comment). For small errors e_att ~ minus the rotation vector from
    // desired to actual attitude.
    const Mat3 R_actual = estimated_state.orientation.to_dcm();
    const Mat3 R_err = R_des.transpose() * R_actual;
    last_attitude_error_ = 0.5 * vee(R_err.transpose() - R_err);

    // Rate error: pure damping in Phase 2 (omega_des defaults to zero).
    const Vec3 rate_error = omega_desired_ - estimated_state.angular_rate;

    ControlCommand cmd;
    cmd.collective_thrust = thrust;
    cmd.moment = gains_.kp_att.cwiseProduct(last_attitude_error_) +
                 gains_.kd_att.cwiseProduct(rate_error);
    return cmd;
}

inline void CascadedController::reset() {
    omega_desired_.setZero();
    integral_pos_.setZero();
    last_a_des_.setZero();
    last_attitude_error_.setZero();
}

inline void CascadedController::set_omega_desired(const Vec3& omega_desired) {
    omega_desired_ = omega_desired;
}

}  // namespace talon
