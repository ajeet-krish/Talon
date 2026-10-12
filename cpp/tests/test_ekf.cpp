#include <cmath>
#include <iostream>
#include <random>
#include <vector>

#include <gtest/gtest.h>

#include "talon/constants.hpp"
#include "talon/dynamics.hpp"
#include "talon/ekf.hpp"
#include "talon/types.hpp"

using talon::EKFConfig;
using talon::EKFNominalState;
using talon::Ekf;
using talon::Mat3;
using talon::Matrix15;
using talon::Quaternion;
using talon::Vec3;

namespace {

constexpr double kDeg = 0.017453292519943295;

// Low-noise config for the deterministic (perfect-sensor) convergence tests:
// tiny IMU densities and walks, tight GPS R. Keeps the acceptance thresholds
// robust against platform-level RNG variance while still exercising the full
// predict/update code path.
EKFConfig quiet_config() {
    EKFConfig c;
    c.accel_noise_density = 1e-4;
    c.gyro_noise_density = 1e-4;
    c.accel_bias_walk = 1e-8;
    c.gyro_bias_walk = 1e-8;
    c.gps_pos_noise = 0.05;
    c.gps_pos_noise_vertical = 0.05;
    c.gps_vel_noise = 0.01;
    return c;
}

// Hamilton quaternion for a rotation of `angle` about `axis` (unit).
Quaternion quat_from_axis_angle(double angle, const Vec3& axis) {
    return talon::exp_quat(axis * angle);
}

// Roll angle extracted from a body-to-NED DCM (aerospace Tait-Bryan).
double roll_from_quat(const Quaternion& q) {
    const Mat3 R = q.to_dcm();
    return std::atan2(R(2, 1), R(2, 2));
}

double pitch_from_quat(const Quaternion& q) {
    const Mat3 R = q.to_dcm();
    return std::asin(-R(2, 0));
}

double yaw_from_quat(const Quaternion& q) {
    const Mat3 R = q.to_dcm();
    return std::atan2(R(1, 0), R(0, 0));
}

// Perfect hover IMU truth: stationary level vehicle reads the gravity
// reaction [0, 0, -g] and zero body rate (docs/conventions.md).
const Vec3 kHoverAccel(0.0, 0.0, -talon::GRAVITY);
const Vec3 kHoverGyro = Vec3::Zero();

}  // namespace

TEST(EKF, TruthInjectionPositionConvergence) {
    // EKF starts at a wrong position (up to 5 m offset) but with perfect
    // IMU (static hover) and perfect GPS at the origin. With the quiet
    // config the position must converge well inside 0.1 m in 10 s.
    Ekf ekf(quiet_config());
    EKFNominalState init;
    init.position = Vec3(5.0, -3.0, -12.0);
    init.velocity = Vec3(0.3, -0.2, 0.1);
    ekf.initialize(init);

    const int steps = static_cast<int>(10.0 / talon::DT_IMU);
    for (int i = 0; i < steps; ++i) {
        ekf.predict(kHoverAccel, kHoverGyro, talon::DT_IMU);
        if (i % 20 == 0) {  // DT_GPS = 0.1 s
            ekf.update(Vec3::Zero(), Vec3::Zero());
        }
    }

    EXPECT_LT(ekf.nominal().position.norm(), 0.1);
    EXPECT_LT(ekf.nominal().velocity.norm(), 0.05);
    EXPECT_TRUE(ekf.last_update_accepted());
}

TEST(EKF, AttitudeErrorVelocityDriftSignPin) {
    // Pins the NOMINAL propagation sign chain: for a +roll attitude error
    // (right/body-frame error convention), one predict step must attribute
    // the hover specific force [0, 0, -g] to a lateral NED acceleration
    // a_y = g * sin(+roll) > 0 (and a_z = g*(1 - cos(roll))). This pins the
    // a_ned = R * f_body + g sign chain used in the nominal propagation, not
    // the dv/dtheta Jacobian sign; the Jacobian sign is guarded by the
    // convergence tests above and the noisy-simulation tests below.
    Ekf ekf(quiet_config());
    EKFNominalState init;
    init.attitude = quat_from_axis_angle(0.01, Vec3::UnitX());
    ekf.initialize(init);

    ekf.predict(kHoverAccel, kHoverGyro, talon::DT_IMU);

    const double phi = 0.01;
    EXPECT_NEAR(ekf.nominal().velocity.y(),
                talon::GRAVITY * std::sin(phi) * talon::DT_IMU, 1e-12);
    EXPECT_NEAR(ekf.nominal().velocity.x(), 0.0, 1e-12);
    EXPECT_NEAR(ekf.nominal().velocity.z(),
                talon::GRAVITY * (1.0 - std::cos(phi)) * talon::DT_IMU, 1e-12);
}

TEST(EKF, NoisySteadyStateHover) {
    // 100 s static hover with default-config IMU (densities) and GPS noise,
    // truth biases zero, fixed RNG seed. RMSE is accumulated over every GPS
    // epoch of the whole run (no transient: truth starts at the EKF init).
    // Acceptance: position RMSE < 2.0 m, velocity RMSE < 0.2 m/s (achieved
    // values are reported in the test output; the draft 1.5 m bound was
    // relaxed because with sigma_gps_pos = 1.5 m the steady-state position
    // error floor sits near ~1 m RMS per axis).
    std::mt19937 rng(20261012u);
    std::normal_distribution<double> normal(0.0, 1.0);

    const EKFConfig config;  // production defaults
    Ekf ekf(config);
    ekf.initialize(EKFNominalState{});

    const double accel_sigma =
        config.accel_noise_density / std::sqrt(talon::DT_IMU);
    const double gyro_sigma =
        config.gyro_noise_density / std::sqrt(talon::DT_IMU);

    double pos_sq_sum = 0.0;
    double vel_sq_sum = 0.0;
    int gps_count = 0;
    int gated_count = 0;
    const int steps = static_cast<int>(100.0 / talon::DT_IMU);
    for (int i = 0; i < steps; ++i) {
        const Vec3 accel =
            kHoverAccel + accel_sigma * Vec3(normal(rng), normal(rng), normal(rng));
        const Vec3 gyro =
            kHoverGyro + gyro_sigma * Vec3(normal(rng), normal(rng), normal(rng));
        ekf.predict(accel, gyro, talon::DT_IMU);
        if (i % 20 == 0) {
            const Vec3 gps_pos(
                config.gps_pos_noise * normal(rng),
                config.gps_pos_noise * normal(rng),
                config.gps_pos_noise_vertical * normal(rng));
            const Vec3 gps_vel(
                config.gps_vel_noise * normal(rng),
                config.gps_vel_noise * normal(rng),
                config.gps_vel_noise * normal(rng));
            // Occasional gate rejections are EXPECTED at ~5-sigma with 6 DOF
            // (P(chi2 > 25) ~ 1e-3 per epoch over 1000 epochs): the filter is
            // working as designed, so count them instead of asserting.
            if (ekf.update(gps_pos, gps_vel)) {
                pos_sq_sum += ekf.nominal().position.squaredNorm();
                vel_sq_sum += ekf.nominal().velocity.squaredNorm();
                ++gps_count;
            } else {
                ++gated_count;
            }
        }
    }

    ASSERT_GT(gps_count, 0);
    const double pos_rmse = std::sqrt(pos_sq_sum / gps_count);
    const double vel_rmse = std::sqrt(vel_sq_sum / gps_count);
    std::cout << "[NoisySteadyStateHover] position RMSE = " << pos_rmse
              << " m, velocity RMSE = " << vel_rmse << " m/s over "
              << gps_count << " accepted GPS epochs (" << gated_count
              << " gated out)\n";
    EXPECT_LT(pos_rmse, 2.0);
    EXPECT_LT(vel_rmse, 0.2);
    // The gate must not be firing constantly (that would mean inconsistent
    // noise settings, not a rare tail event).
    EXPECT_LT(gated_count, 50);
}

TEST(EKF, GyroBiasObservabilityWithPitchProfile) {
    // Gyro bias estimation with a prescribed slow pitch oscillation
    // theta(t) = A * sin(w * t) (a "tethered" truth trajectory, a_ned = 0).
    //
    // Why the profile: at a pure static hover the z (yaw) gyro bias is
    // UNOBSERVABLE with position/velocity GPS only - the yaw channel has no
    // gravity reference (gravity constrains roll/pitch through the specific
    // force, and yaw error produces no velocity error when f_body stays
    // [0, 0, -g]). Pitching the vehicle makes f_body = [g sin(theta), 0,
    // -g cos(theta)] time-varying, which couples dtheta_z (driven by
    // -dbg_z) into the observable velocity channel via -R * skew(f) and
    // makes all three gyro bias axes observable. Zero gyro noise + perfect
    // GPS isolates the observability question from the noise floor.
    const double A = 0.25;   // rad pitch amplitude
    const double w = 1.0;    // rad/s pitch rate
    const Vec3 true_bias(0.01, -0.005, 0.008);

    Ekf ekf(quiet_config());
    ekf.initialize(EKFNominalState{});

    const int steps = static_cast<int>(30.0 / talon::DT_IMU);
    for (int i = 0; i < steps; ++i) {
        const double t = i * talon::DT_IMU;
        const double theta = A * std::sin(w * t);
        const double theta_dot = A * w * std::cos(w * t);
        const Vec3 omega(0.0, theta_dot, 0.0);  // body rate for pitch about y
        const Vec3 f_body(talon::GRAVITY * std::sin(theta), 0.0,
                          -talon::GRAVITY * std::cos(theta));
        ekf.predict(f_body, omega + true_bias, talon::DT_IMU);
        if (i % 20 == 0) {
            ASSERT_TRUE(ekf.update(Vec3::Zero(), Vec3::Zero()));
        }
    }

    const Vec3 est = ekf.nominal().gyro_bias;
    std::cout << "[GyroBias] true = [" << true_bias.transpose()
              << "], estimated = [" << est.transpose() << "]\n";
    for (int i = 0; i < 3; ++i) {
        EXPECT_NEAR(est[i], true_bias[i], 0.2 * std::abs(true_bias[i]))
            << "gyro bias axis " << i << " off by more than 20%";
    }
}

TEST(EKF, CovarianceStaysSymmetricPositiveDefinite) {
    // 100 s noisy hover (same setup as NoisySteadyStateHover): P must stay
    // symmetric to 1e-12 and strictly positive definite at the end and at
    // 10 s spot checks (Joseph-form reset + explicit symmetrization).
    std::mt19937 rng(7u);
    std::normal_distribution<double> normal(0.0, 1.0);

    const EKFConfig config;
    Ekf ekf(config);
    ekf.initialize(EKFNominalState{});

    const double accel_sigma =
        config.accel_noise_density / std::sqrt(talon::DT_IMU);
    const double gyro_sigma =
        config.gyro_noise_density / std::sqrt(talon::DT_IMU);

    const int steps = static_cast<int>(100.0 / talon::DT_IMU);
    for (int i = 0; i < steps; ++i) {
        const Vec3 accel =
            kHoverAccel + accel_sigma * Vec3(normal(rng), normal(rng), normal(rng));
        const Vec3 gyro =
            kHoverGyro + gyro_sigma * Vec3(normal(rng), normal(rng), normal(rng));
        ekf.predict(accel, gyro, talon::DT_IMU);
        if (i % 20 == 0) {
            const Vec3 gps_pos(
                config.gps_pos_noise * normal(rng),
                config.gps_pos_noise * normal(rng),
                config.gps_pos_noise_vertical * normal(rng));
            const Vec3 gps_vel(
                config.gps_vel_noise * normal(rng),
                config.gps_vel_noise * normal(rng),
                config.gps_vel_noise * normal(rng));
            ekf.update(gps_pos, gps_vel);  // rare gate-outs are fine here
        }
        if (i % 2000 == 0 && i > 0) {  // every 10 s
            const Matrix15 P = ekf.covariance();
            EXPECT_LT((P - P.transpose()).lpNorm<Eigen::Infinity>(), 1e-12);
            Eigen::SelfAdjointEigenSolver<Matrix15> solver(P);
            EXPECT_GT(solver.eigenvalues().minCoeff(), 0.0);
        }
    }

    const Matrix15 P = ekf.covariance();
    EXPECT_LT((P - P.transpose()).lpNorm<Eigen::Infinity>(), 1e-12);
    Eigen::SelfAdjointEigenSolver<Matrix15> solver(P);
    EXPECT_GT(solver.eigenvalues().minCoeff(), 0.0);
}

TEST(EKF, InnovationGatingRejectsOutlier) {
    // Converge on perfect sensors, then feed a GPS position 50 m off: the
    // update must be rejected, the chi2 must exceed the gate, and the
    // nominal state must not move.
    const EKFConfig config = quiet_config();
    Ekf ekf(config);
    ekf.initialize(EKFNominalState{});

    const int warmup = static_cast<int>(10.0 / talon::DT_IMU);
    for (int i = 0; i < warmup; ++i) {
        ekf.predict(kHoverAccel, kHoverGyro, talon::DT_IMU);
        if (i % 20 == 0) {
            ASSERT_TRUE(ekf.update(Vec3::Zero(), Vec3::Zero()));
        }
    }

    const EKFNominalState before = ekf.nominal();
    const bool accepted =
        ekf.update(Vec3(50.0, 0.0, 0.0), Vec3::Zero());
    EXPECT_FALSE(accepted);
    EXPECT_FALSE(ekf.last_update_accepted());
    EXPECT_GT(ekf.last_innovation_chi2(), config.innovation_gate_chi2);
    EXPECT_EQ(ekf.nominal().position, before.position);
    EXPECT_EQ(ekf.nominal().velocity, before.velocity);
    EXPECT_EQ(ekf.nominal().gyro_bias, before.gyro_bias);
    EXPECT_EQ(ekf.nominal().accel_bias, before.accel_bias);

    // The next clean fix is still accepted (the filter is not stuck).
    EXPECT_TRUE(ekf.update(Vec3::Zero(), Vec3::Zero()));
    EXPECT_TRUE(ekf.last_update_accepted());
}

TEST(EKF, AttitudeErrorInjectionConvergence) {
    // Start with a 5 deg roll error and perfect sensors (static hover truth,
    // GPS at the origin); the roll estimate must fall inside 0.5 deg in 20 s.
    // Roll is observable at hover through the gravity reaction: a roll error
    // tilts the [0, 0, -g] specific force into a lateral NED acceleration
    // that GPS velocity exposes (see the drift-sign pin above).
    const double phi0 = 5.0 * kDeg;
    Ekf ekf(quiet_config());
    EKFNominalState init;
    init.attitude = quat_from_axis_angle(phi0, Vec3::UnitX());
    ekf.initialize(init);

    const int steps = static_cast<int>(20.0 / talon::DT_IMU);
    for (int i = 0; i < steps; ++i) {
        ekf.predict(kHoverAccel, kHoverGyro, talon::DT_IMU);
        if (i % 20 == 0) {
            ASSERT_TRUE(ekf.update(Vec3::Zero(), Vec3::Zero()));
        }
    }

    EXPECT_LT(std::abs(roll_from_quat(ekf.nominal().attitude)), 0.5 * kDeg);
    EXPECT_LT(std::abs(pitch_from_quat(ekf.nominal().attitude)), 0.5 * kDeg);
}

TEST(EKF, HighDynamicsNoNaN) {
    // Free-fall (zero specific force) with large tumbling body rates: the
    // nominal state and P must stay finite (no quaternion blow-up, no
    // covariance overflow) over 10 s of pure prediction.
    Ekf ekf(EKFConfig{});
    ekf.initialize(EKFNominalState{});

    const Vec3 big_gyro(5.0, -3.0, 8.0);
    const int steps = static_cast<int>(10.0 / talon::DT_IMU);
    for (int i = 0; i < steps; ++i) {
        ekf.predict(Vec3::Zero(), big_gyro, talon::DT_IMU);
    }

    const EKFNominalState& s = ekf.nominal();
    EXPECT_TRUE(s.position.allFinite());
    EXPECT_TRUE(s.velocity.allFinite());
    EXPECT_TRUE(s.gyro_bias.allFinite());
    EXPECT_TRUE(s.accel_bias.allFinite());
    EXPECT_NEAR(s.attitude.norm(), 1.0, 1e-9);
    EXPECT_TRUE(ekf.covariance().allFinite());

    // Also with a huge thrust-direction specific force (e.g. 100 g hit).
    ekf.predict(Vec3(0.0, 0.0, -100.0 * talon::GRAVITY), big_gyro,
                 talon::DT_IMU);
    EXPECT_TRUE(ekf.nominal().position.allFinite());
    EXPECT_TRUE(ekf.covariance().allFinite());
}

TEST(EKF, InitializeFromAccelLevelsRollPitch) {
    // A stationary tilted vehicle's specific force [g sin(th), 0, -g cos(th)]
    // must level the filter to (roll = 0, pitch = th); yaw passes through.
    const double pitch_true = 7.0 * kDeg;
    const Vec3 accel(talon::GRAVITY * std::sin(pitch_true), 0.0,
                     -talon::GRAVITY * std::cos(pitch_true));
    Ekf ekf(quiet_config());
    ekf.initialize_from_accel(Vec3(1.0, -2.0, -3.0), Vec3::Zero(), accel,
                              /*yaw=*/0.5);

    EXPECT_NEAR(pitch_from_quat(ekf.nominal().attitude), pitch_true, 1e-9);
    EXPECT_NEAR(roll_from_quat(ekf.nominal().attitude), 0.0, 1e-9);
    EXPECT_NEAR(yaw_from_quat(ekf.nominal().attitude), 0.5, 1e-9);
    EXPECT_EQ(ekf.nominal().position, Vec3(1.0, -2.0, -3.0));
}

TEST(EKF, EstimatedBodyRatesSubtractGyroBias) {
    Ekf ekf(quiet_config());
    EKFNominalState init;
    init.gyro_bias = Vec3(0.01, -0.02, 0.005);
    ekf.initialize(init);
    const Vec3 gyro_meas(0.5, -0.3, 0.25);
    EXPECT_EQ(ekf.estimated_body_rates(gyro_meas),
              gyro_meas - Vec3(0.01, -0.02, 0.005));
}
