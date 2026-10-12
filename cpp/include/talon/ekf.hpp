#pragma once

#include <algorithm>
#include <cmath>

#include <Eigen/Dense>

#include "talon/constants.hpp"
#include "talon/dynamics.hpp"
#include "talon/types.hpp"

namespace talon {

// 15-state multiplicative error-state Kalman filter (MEKF) for INS/GPS
// navigation. References: Trawny & Roumeliotis 2005 ("Indirect Kalman Filter
// for 3D Attitude Estimation"), Sola 2017 ("Quaternion kinematics for the
// error-state Kalman filter", arXiv:1711.02508).
//
// Error-state vector ordering (FIXED; do not reorder without updating H, the
// injection, and the tests):
//
//   dx = [dp(3), dv(3), dtheta(3), dbg(3), dba(3)]
//     dp     position error, NED (m)
//     dv     velocity error, NED (m/s)
//     dtheta attitude error (rad, 3-vector)
//     dbg    gyro bias error (rad/s, body)
//     dba    accel bias error (m/s^2, body)
//
// Attitude error convention: BODY-FRAME (right) multiplicative error,
//
//   q_true = q_est (x) dq,   dq = [cos(|dtheta|/2), sin(|dtheta|/2)
//                                  * dtheta/|dtheta|],   dq ~ [1, 0.5*dtheta]
//
// i.e. dtheta is expressed in the ESTIMATED body frame (Sola 2017
// "body-frame error", section 4.2; the same composition order as Trawny &
// Roumeliotis eq. 22-23). This is the natural partner of the controller's
// body-rate usage (docs/conventions.md: q_dot = 0.5 * q * [0, omega_body],
// body-frame rates), because the attitude error then obeys the body-frame
// kinematics dtheta_dot = -[omega]_x * dtheta - dbg + noise, with the same
// omega = gyro_meas - gyro_bias that propagates the nominal quaternion.
// Nominal attitude increments are therefore RIGHT-multiplied:
// q <- q (x) exp_quat(omega * dt).
//
// Continuous-time error dynamics (ideal, f = accel_meas - ba, R = R(q_est)):
//   dp_dot     = dv
//   dv_dot     = -R * [f]_x * dtheta - R * dba + n_a      (Sola eq. 280 body
//                frame: the + sign pitfall is avoided by deriving from
//                q_true = q_est (x) dq: a_true = R(dq) R (f - dba) + g
//                ~ R f + R [dtheta]_x f - R dba + g = a_est - R [f]_x dtheta
//                - R dba, so the dtheta coefficient is -R [f]_x)
//   dtheta_dot = -[omega]_x dtheta - dbg + n_g
//   dbg_dot    = n_bgw,  dba_dot = n_baw   (random walks)
//
// The discretized Jacobian F below is first order in dt. Documented
// simplifications:
// - The nominal p, v update is Euler (not exact double-integrator); at
//   DT_IMU = 0.005 s the neglected term is 0.5*a*dt^3 per step.
// - Qd uses the standard first-order drives: the position block
//   0.25 * sigma_a^2 * dt^3 (trapezoidal-consistent approximation of the
//   exact white-noise result sigma_a^2 * dt^3 / 3) with the matching
//   0.5 * sigma_a^2 * dt^2 position-velocity cross block.
//
// Gravity: fixed talon::GRAVITY (NED g_ned = [0, 0, +g]); the accelerometer
// measures specific force, so g is added exactly once in predict()
// (a_ned = R * f_body + g_ned), matching docs/conventions.md.
struct EKFConfig {
    // Continuous-time noise densities (per sqrt(Hz)) - defaults from
    // ADIS16448-class IMU.
    double accel_noise_density = 0.004;   // m/s^2/sqrt(Hz)
    double gyro_noise_density = 0.003;    // rad/s/sqrt(Hz)
    double accel_bias_walk = 5e-4;        // m/s^2/sqrt(s)
    double gyro_bias_walk = 1e-5;         // rad/s/sqrt(s)
    // GPS (discrete, used in update R)
    double gps_pos_noise = 1.5;           // m 1-sigma horizontal
    double gps_pos_noise_vertical = 3.0;  // m 1-sigma vertical
    double gps_vel_noise = 0.1;           // m/s 1-sigma
    // Gating
    double innovation_gate_chi2 = 25.0;   // ~5-sigma, 6 DOF
    // Initial covariance diagonals (std devs)
    double init_pos_std = 5.0;
    double init_vel_std = 1.0;
    double init_att_std = 0.1;            // rad (~5.7 deg)
    double init_gyro_bias_std = 0.01;     // rad/s
    double init_accel_bias_std = 0.1;     // m/s^2
};

// Nominal (error-free) state of the MEKF: position/velocity in NED, attitude
// as a Hamilton body-to-NED quaternion, biases in the body frame.
struct EKFNominalState {
    Vec3 position = Vec3::Zero();      // NED m
    Vec3 velocity = Vec3::Zero();      // NED m/s
    Quaternion attitude;               // Hamilton [w,x,y,z], body-to-NED
    Vec3 gyro_bias = Vec3::Zero();     // rad/s body
    Vec3 accel_bias = Vec3::Zero();    // m/s^2 body
};

using Matrix15 = Eigen::Matrix<double, 15, 15>;
using Matrix6x15 = Eigen::Matrix<double, 6, 15>;
using Vector15 = Eigen::Matrix<double, 15, 1>;
using Vector6 = Eigen::Matrix<double, 6, 1>;

// Skew-symmetric (cross-product) matrix [v]_x with v x w = [v]_x * w.
// Inverse of the controller's vee() (controller.hpp).
inline Mat3 skew(const Vec3& v) {
    Mat3 m;
    m << 0.0, -v.z(), v.y(),
         v.z(), 0.0, -v.x(),
         -v.y(), v.x(), 0.0;
    return m;
}

// Rotation vector -> unit quaternion, exp_quat(phi) with phi the FULL rotation
// vector (angle*axis): q = [cos(|phi|/2), sin(|phi|/2) * phi/|phi|]. Series
// expansion for |phi| ~ 0: q ~ [1, phi/2].
inline Quaternion exp_quat(const Vec3& phi) {
    const double angle = phi.norm();
    if (angle < 1e-8) {
        return Quaternion{1.0, 0.5 * phi.x(), 0.5 * phi.y(), 0.5 * phi.z()}
            .normalized();
    }
    const double half = 0.5 * angle;
    const double s = std::sin(half) / angle;
    return Quaternion{std::cos(half), s * phi.x(), s * phi.y(), s * phi.z()};
}

// 15-state MEKF. Call predict() every DT_IMU = 0.005 s with raw IMU samples
// (accel: specific force, body frame; gyro: body rates) and update() every
// DT_GPS = 0.1 s with GPS position/velocity in NED.
class Ekf {
public:
    explicit Ekf(const EKFConfig& config = EKFConfig{})
        : config_(config), P_(Matrix15::Zero()) {
        reset_telemetry();
    }

    // Initialize the nominal state and set P to the configured initial
    // covariances (diagonal).
    void initialize(const EKFNominalState& initial) {
        nominal_ = initial;
        nominal_.attitude = nominal_.attitude.normalized();
        P_.setZero();
        P_.block<3, 3>(0, 0) =
            config_.init_pos_std * config_.init_pos_std *
            Mat3::Identity();
        P_.block<3, 3>(3, 3) =
            config_.init_vel_std * config_.init_vel_std *
            Mat3::Identity();
        P_.block<3, 3>(6, 6) =
            config_.init_att_std * config_.init_att_std *
            Mat3::Identity();
        P_.block<3, 3>(9, 9) =
            config_.init_gyro_bias_std * config_.init_gyro_bias_std *
            Mat3::Identity();
        P_.block<3, 3>(12, 12) =
            config_.init_accel_bias_std * config_.init_accel_bias_std *
            Mat3::Identity();
        reset_telemetry();
    }

    // Convenience: initialize attitude level (roll = pitch = 0) from the
    // assumed stationary specific force [0, 0, -g] (docs/conventions.md
    // hover reading), yaw as provided; position/velocity from args.
    void initialize_level(const Vec3& position, const Vec3& velocity,
                          double yaw = 0.0) {
        initialize_from_accel(position, velocity, Vec3(0.0, 0.0, -GRAVITY),
                              yaw);
    }

    // Like initialize_level() but levels roll/pitch from a measured
    // stationary specific force (accel_body ~ [0, 0, -g] when level):
    //   accel = [ g*sin(pitch),
    //            -g*sin(roll)*cos(pitch),
    //            -g*cos(roll)*cos(pitch) ]
    // => pitch = asin(ax / g), roll = atan2(-ay, -az).
    void initialize_from_accel(const Vec3& position, const Vec3& velocity,
                               const Vec3& accel_body, double yaw = 0.0) {
        EKFNominalState initial;
        initial.position = position;
        initial.velocity = velocity;
        const double ax_g =
            std::clamp(accel_body.x() / GRAVITY, -1.0, 1.0);
        const double pitch = std::asin(ax_g);
        const double roll = std::atan2(-accel_body.y(), -accel_body.z());
        const Quaternion qx{std::cos(0.5 * roll), std::sin(0.5 * roll), 0.0,
                            0.0};
        const Quaternion qy{std::cos(0.5 * pitch), 0.0, std::sin(0.5 * pitch),
                            0.0};
        const Quaternion qz{std::cos(0.5 * yaw), 0.0, 0.0,
                            std::sin(0.5 * yaw)};
        initial.attitude =
            quat_normalize(quat_multiply(quat_multiply(qz, qy), qx));
        initialize(initial);
    }

    // IMU propagation (call every DT_IMU = 0.005 s). accel_meas is the
    // specific force in body frame (includes the gravity reaction: hover
    // reads [0, 0, -g]); gyro_meas is the body angular rate.
    void predict(const Vec3& accel_meas, const Vec3& gyro_meas, double dt) {
        // Bias-corrected IMU quantities.
        const Vec3 omega = gyro_meas - nominal_.gyro_bias;
        const Vec3 f_body = accel_meas - nominal_.accel_bias;
        const Mat3 R = rotation_matrix(nominal_.attitude);

        // Nominal translation: specific force + gravity, added exactly once.
        const Vec3 a_ned = R * f_body + Vec3(0.0, 0.0, GRAVITY);
        nominal_.position += nominal_.velocity * dt + 0.5 * a_ned * dt * dt;
        nominal_.velocity += a_ned * dt;

        // Nominal attitude: body-frame increment, right-multiplied to match
        // the body-frame error convention (q_true = q_est (x) dq).
        nominal_.attitude =
            quat_normalize(quat_multiply(nominal_.attitude,
                                         exp_quat(omega * dt)));

        // Biases are constant in the nominal model (random walk enters
        // through Qd only).

        // Error-state Jacobian F = I + A*dt (first order).
        Matrix15 F = Matrix15::Identity();
        F.block<3, 3>(0, 3) = Mat3::Identity() * dt;              // dp / dv
        F.block<3, 3>(3, 6) = -R * skew(f_body) * dt;             // dv / dtheta
        F.block<3, 3>(3, 12) = -R * dt;                           // dv / dba
        F.block<3, 3>(6, 6) =
            Mat3::Identity() - skew(omega) * dt;                  // dtheta
        F.block<3, 3>(6, 9) = -Mat3::Identity() * dt;             // dtheta/dbg

        // Discrete process noise (diagonal drives plus the correlated
        // position-velocity drive from the accelerometer noise).
        const double sa2 =
            config_.accel_noise_density * config_.accel_noise_density;
        const double sg2 =
            config_.gyro_noise_density * config_.gyro_noise_density;
        const double sbg2 = config_.gyro_bias_walk * config_.gyro_bias_walk;
        const double sba2 = config_.accel_bias_walk * config_.accel_bias_walk;
        Matrix15 Qd = Matrix15::Zero();
        Qd.block<3, 3>(0, 0) = 0.25 * sa2 * dt * dt * dt * Mat3::Identity();
        Qd.block<3, 3>(0, 3) = 0.5 * sa2 * dt * dt * Mat3::Identity();
        Qd.block<3, 3>(3, 0) = Qd.block<3, 3>(0, 3).transpose();
        Qd.block<3, 3>(3, 3) = sa2 * dt * Mat3::Identity();
        Qd.block<3, 3>(6, 6) = sg2 * dt * Mat3::Identity();
        Qd.block<3, 3>(9, 9) = sbg2 * dt * Mat3::Identity();
        Qd.block<3, 3>(12, 12) = sba2 * dt * Mat3::Identity();

        P_ = F * P_ * F.transpose() + Qd;
        P_ = 0.5 * (P_ + P_.transpose());  // fight round-off asymmetry
    }

    // GPS update (call every DT_GPS = 0.1 s). Returns true if accepted,
    // false if gated out (in which case the state and covariance are NOT
    // modified).
    bool update(const Vec3& gps_position, const Vec3& gps_velocity) {
        // Innovation: GPS observes dp and dv directly (H = [I6 | 0]).
        Vector6 y;
        y.head<3>() = gps_position - nominal_.position;
        y.tail<3>() = gps_velocity - nominal_.velocity;

        Matrix6x15 H = Matrix6x15::Zero();
        H.block<6, 6>(0, 0) = Eigen::Matrix<double, 6, 6>::Identity();

        Eigen::Matrix<double, 6, 6> R_meas =
            Eigen::Matrix<double, 6, 6>::Zero();
        R_meas(0, 0) = config_.gps_pos_noise * config_.gps_pos_noise;
        R_meas(1, 1) = config_.gps_pos_noise * config_.gps_pos_noise;
        R_meas(2, 2) =
            config_.gps_pos_noise_vertical * config_.gps_pos_noise_vertical;
        R_meas(3, 3) = config_.gps_vel_noise * config_.gps_vel_noise;
        R_meas(4, 4) = config_.gps_vel_noise * config_.gps_vel_noise;
        R_meas(5, 5) = config_.gps_vel_noise * config_.gps_vel_noise;

        const Eigen::Matrix<double, 6, 6> S =
            H * P_ * H.transpose() + R_meas;
        const double chi2 = y.dot(S.ldlt().solve(y));
        last_innovation_ = y;
        last_innovation_chi2_ = chi2;

        // Gate BEFORE applying: a rejected fix leaves state and covariance
        // untouched.
        if (chi2 > config_.innovation_gate_chi2) {
            last_update_accepted_ = false;
            return false;
        }

        const Eigen::Matrix<double, 15, 6> K =
            P_ * H.transpose() * S.inverse();  // 6x6, R keeps S well posed
        const Vector15 dx = K * y;

        // Inject the error into the nominal state, then reset the error
        // state to zero (implicitly, by construction).
        nominal_.position += dx.head<3>();
        nominal_.velocity += dx.segment<3>(3);
        nominal_.attitude = quat_normalize(
            quat_multiply(nominal_.attitude, exp_quat(dx.segment<3>(6))));
        nominal_.gyro_bias += dx.segment<3>(9);
        nominal_.accel_bias += dx.segment<3>(12);

        // Covariance reset with the Joseph form (keeps P symmetric PD).
        const Matrix15 G = Matrix15::Identity() - K * H;
        P_ = G * P_ * G.transpose() + K * R_meas * K.transpose();
        P_ = 0.5 * (P_ + P_.transpose());

        last_update_accepted_ = true;
        return true;
    }

    // Accessors.
    const EKFNominalState& nominal() const { return nominal_; }
    Matrix15 covariance() const { return P_; }
    Vector6 last_innovation() const { return last_innovation_; }
    double last_innovation_chi2() const { return last_innovation_chi2_; }
    bool last_update_accepted() const { return last_update_accepted_; }

    // For testing: direct state access (fault injection hook, Phase 5).
    EKFNominalState& mutable_nominal() { return nominal_; }
    Matrix15& mutable_covariance() { return P_; }

    // Estimated body rates from the last gyro sample: gyro_meas - gyro_bias.
    // The EKF does not estimate angular rate separately; the caller passes
    // the raw gyro measurement (State.angular_rate view for the controller).
    Vec3 estimated_body_rates(const Vec3& gyro_meas) const {
        return gyro_meas - nominal_.gyro_bias;
    }

private:
    void reset_telemetry() {
        last_innovation_.setZero();
        last_innovation_chi2_ = 0.0;
        last_update_accepted_ = true;
    }

    EKFConfig config_;
    EKFNominalState nominal_;
    Matrix15 P_;
    Vector6 last_innovation_ = Vector6::Zero();
    double last_innovation_chi2_ = 0.0;
    bool last_update_accepted_ = true;
};

}  // namespace talon
