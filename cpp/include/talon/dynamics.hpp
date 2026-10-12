#pragma once

#include "talon/types.hpp"

namespace talon {

// Hamilton quaternion product: a * b.
inline Quaternion quat_multiply(const Quaternion& a, const Quaternion& b) {
    return Quaternion{
        a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z,
        a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
        a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
        a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w,
    };
}

inline Quaternion quat_normalize(const Quaternion& q) {
    return q.normalized();
}

// Quaternion derivative for body-frame angular rate:
// q_dot = 0.5 * q * [0, omega_body].
inline Quaternion quat_derivative(const Quaternion& q, const Vec3& omega_body) {
    const Quaternion omega_q{0.0, omega_body.x(), omega_body.y(), omega_body.z()};
    const Quaternion qd = quat_multiply(q, omega_q);
    return Quaternion{0.5 * qd.w, 0.5 * qd.x, 0.5 * qd.y, 0.5 * qd.z};
}

// Body-to-NED direction cosine matrix from a Hamilton quaternion.
inline Mat3 rotation_matrix(const Quaternion& q) {
    return q.to_dcm();
}

// Componentwise state update r = s + a * d (quaternion treated as 4-vector).
// Used only for intermediate RK4 stages; the final quaternion is renormalized.
inline State state_add_scaled(const State& s, const State& d, double a) {
    State r;
    r.position = s.position + a * d.position;
    r.velocity = s.velocity + a * d.velocity;
    r.angular_rate = s.angular_rate + a * d.angular_rate;
    r.orientation.w = s.orientation.w + a * d.orientation.w;
    r.orientation.x = s.orientation.x + a * d.orientation.x;
    r.orientation.y = s.orientation.y + a * d.orientation.y;
    r.orientation.z = s.orientation.z + a * d.orientation.z;
    return r;
}

// Continuous-time Newton-Euler derivative in NED.
//
// Thrust convention: collective thrust T >= 0 acts along -body-z, so a level
// vehicle (identity orientation) is pushed in -NED-z (upward).
//
// Drag: quadratic, F_drag = -0.5 * rho * Cd * A * ||v_rel|| * v_rel with
// v_rel = v_ground_ned - v_wind_ned.
//
// Angular: omega_dot = I^-1 * (M - omega x (I * omega)).
inline State state_derivative(const State& s, double thrust, const Vec3& moment,
                              const Vec3& wind_ned, const VehicleParams& p) {
    State d;
    d.position = s.velocity;

    const Mat3 R = rotation_matrix(s.orientation);
    const Vec3 thrust_body(0.0, 0.0, -thrust);
    const Vec3 v_rel = s.velocity - wind_ned;
    const double k_drag = 0.5 * p.air_density * p.drag_coeff * p.vehicle_area;
    d.velocity = (R * thrust_body) / p.mass +
                 Vec3(0.0, 0.0, p.gravity) -
                 (k_drag * v_rel.norm() * v_rel) / p.mass;

    d.orientation = quat_derivative(s.orientation, s.angular_rate);

    const Vec3 inertia = p.inertia_diag;
    const Vec3 i_omega = inertia.cwiseProduct(s.angular_rate);
    const Vec3 gyro = s.angular_rate.cross(i_omega);
    d.angular_rate = (moment - gyro).cwiseQuotient(inertia);
    return d;
}

// Integrate one RK4 step of the 13-state [pos(3), quat(4), vel(3), omega(3)]
// model. Quaternion is renormalized at the end of the step.
inline State propagate(const State& state, double thrust, const Vec3& moment,
                       const Vec3& wind_ned, double dt,
                       const VehicleParams& params) {
    const State k1 = state_derivative(state, thrust, moment, wind_ned, params);
    const State k2 = state_derivative(state_add_scaled(state, k1, 0.5 * dt),
                                      thrust, moment, wind_ned, params);
    const State k3 = state_derivative(state_add_scaled(state, k2, 0.5 * dt),
                                      thrust, moment, wind_ned, params);
    const State k4 = state_derivative(state_add_scaled(state, k3, dt),
                                      thrust, moment, wind_ned, params);

    State out;
    out.position = state.position + (dt / 6.0) * (k1.position + 2.0 * k2.position +
                                                  2.0 * k3.position + k4.position);
    out.velocity = state.velocity + (dt / 6.0) * (k1.velocity + 2.0 * k2.velocity +
                                                  2.0 * k3.velocity + k4.velocity);
    out.angular_rate =
        state.angular_rate + (dt / 6.0) * (k1.angular_rate + 2.0 * k2.angular_rate +
                                           2.0 * k3.angular_rate + k4.angular_rate);
    out.orientation.w = state.orientation.w + (dt / 6.0) * (k1.orientation.w + 2.0 * k2.orientation.w +
                                                            2.0 * k3.orientation.w + k4.orientation.w);
    out.orientation.x = state.orientation.x + (dt / 6.0) * (k1.orientation.x + 2.0 * k2.orientation.x +
                                                            2.0 * k3.orientation.x + k4.orientation.x);
    out.orientation.y = state.orientation.y + (dt / 6.0) * (k1.orientation.y + 2.0 * k2.orientation.y +
                                                            2.0 * k3.orientation.y + k4.orientation.y);
    out.orientation.z = state.orientation.z + (dt / 6.0) * (k1.orientation.z + 2.0 * k2.orientation.z +
                                                            2.0 * k3.orientation.z + k4.orientation.z);
    out.orientation = out.orientation.normalized();
    return out;
}

// Thrust needed to hover (level, no acceleration).
inline double hover_thrust(const VehicleParams& params) {
    return params.mass * params.gravity;
}

inline double hover_thrust() {
    return hover_thrust(VehicleParams{});
}

}  // namespace talon
