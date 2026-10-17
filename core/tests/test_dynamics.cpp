#include <cmath>
#include <random>

#include <gtest/gtest.h>

#include "talon/constants.hpp"
#include "talon/dynamics.hpp"
#include "talon/types.hpp"

using talon::Quaternion;
using talon::State;
using talon::Vec3;
using talon::VehicleParams;

namespace {

// State at the origin, at rest, level (identity orientation).
State hover_state() {
    return State{};
}

}  // namespace

TEST(Dynamics, HoverStability) {
    VehicleParams params;
    State state = hover_state();
    const double thrust = talon::hover_thrust(params);
    const Vec3 moment = Vec3::Zero();
    const Vec3 wind = Vec3::Zero();

    const int steps = 12000;  // 60 s at DT = 0.005
    for (int i = 0; i < steps; ++i) {
        state = talon::propagate(state, thrust, moment, wind, talon::DT, params);
    }

    EXPECT_LT(state.position.norm(), 1e-4);
    EXPECT_LT(state.velocity.norm(), 1e-4);
    EXPECT_NEAR(state.orientation.norm(), 1.0, 1e-12);
}

TEST(Dynamics, FreeRotationConservesAngularMomentum) {
    VehicleParams params;
    params.inertia_diag = Vec3(2.0e-3, 3.0e-3, 4.0e-3);
    params.drag_coeff = 0.0;  // decouple translation from attitude

    State state = hover_state();
    state.angular_rate = Vec3(3.0, -2.0, 1.5);
    // For a torque-free triaxial body the BODY-frame momentum I*omega is not
    // constant (it precesses along the polhode); the conserved quantity is the
    // NED-frame momentum R(q) * (I * omega).
    const Vec3 momentum0 =
        state.orientation.rotate_vector(params.inertia_diag.cwiseProduct(state.angular_rate));

    const int steps = static_cast<int>(10.0 / talon::DT);  // 10 s
    for (int i = 0; i < steps; ++i) {
        state = talon::propagate(state, 0.0, Vec3::Zero(), Vec3::Zero(),
                                 talon::DT, params);
    }

    const Vec3 momentum1 =
        state.orientation.rotate_vector(params.inertia_diag.cwiseProduct(state.angular_rate));
    EXPECT_LT((momentum1 - momentum0).norm(), 1e-10);
}

TEST(Dynamics, QuaternionNormLongHorizon) {
    VehicleParams params;
    State state = hover_state();
    const double thrust = talon::hover_thrust(params);

    const int steps = 200000;  // 1000 s at DT = 0.005
    for (int i = 0; i < steps; ++i) {
        state = talon::propagate(state, thrust, Vec3::Zero(), Vec3::Zero(),
                                 talon::DT, params);
    }

    EXPECT_LT(std::abs(state.orientation.norm() - 1.0), 1e-12);
}

TEST(Dynamics, QuaternionNormLongHorizonFreeSpin) {
    // Torque-free triaxial spin: the quaternion is actively integrated for
    // the whole horizon (unlike the static hover case above), so this
    // exercises renormalization under real rotation.
    VehicleParams params;
    params.inertia_diag = Vec3(2.0e-3, 3.0e-3, 4.0e-3);

    State state = hover_state();
    state.angular_rate = Vec3(3.0, -2.0, 1.5);

    const int steps = 200000;  // 1000 s at DT = 0.005
    for (int i = 0; i < steps; ++i) {
        state = talon::propagate(state, 0.0, Vec3::Zero(), Vec3::Zero(),
                                 talon::DT, params);
    }

    EXPECT_LT(std::abs(state.orientation.norm() - 1.0), 1e-12);
    // Guard against the trivially-passing case of a static attitude.
    EXPECT_GT(std::abs(state.orientation.y), 1e-6);
}

TEST(Dynamics, FreeFall) {
    VehicleParams params;
    params.drag_coeff = 0.0;

    State state = hover_state();
    const double sim_time = 2.0;
    const int steps = static_cast<int>(sim_time / talon::DT);
    for (int i = 0; i < steps; ++i) {
        state = talon::propagate(state, 0.0, Vec3::Zero(), Vec3::Zero(),
                                 talon::DT, params);
    }

    // NED: z grows downward, start at rest, thrust 0, no drag.
    const double t = steps * talon::DT;
    EXPECT_NEAR(state.position.z(), 0.5 * params.gravity * t * t, 1e-6);
    EXPECT_NEAR(state.velocity.z(), params.gravity * t, 1e-6);
    EXPECT_NEAR(state.position.x(), 0.0, 1e-12);
    EXPECT_NEAR(state.position.y(), 0.0, 1e-12);
}

TEST(Quaternion, DcmOrthonormality) {
    std::mt19937 rng(123);
    std::normal_distribution<double> normal(0.0, 1.0);

    for (int i = 0; i < 100; ++i) {
        Quaternion q{normal(rng), normal(rng), normal(rng), normal(rng)};
        q = q.normalized();
        const Eigen::Matrix3d R = talon::rotation_matrix(q);
        const Eigen::Matrix3d gram = R * R.transpose();
        EXPECT_LT((gram - Eigen::Matrix3d::Identity()).lpNorm<Eigen::Infinity>(),
                  1e-12);
        EXPECT_NEAR(R.determinant(), 1.0, 1e-12);
    }
}

TEST(Quaternion, MultiplyMatchesComposition) {
    std::mt19937 rng(7);
    std::normal_distribution<double> normal(0.0, 1.0);

    for (int i = 0; i < 100; ++i) {
        Quaternion qa{normal(rng), normal(rng), normal(rng), normal(rng)};
        Quaternion qb{normal(rng), normal(rng), normal(rng), normal(rng)};
        qa = qa.normalized();
        qb = qb.normalized();
        const Vec3 v(normal(rng), normal(rng), normal(rng));

        const Quaternion qab = talon::quat_multiply(qa, qb);
        const Vec3 via_product = qab.rotate_vector(v);
        const Vec3 via_composition = qa.rotate_vector(qb.rotate_vector(v));
        EXPECT_LT((via_product - via_composition).norm(), 1e-12);
    }
}

TEST(Dynamics, HoverThrustValue) {
    const VehicleParams params;
    EXPECT_NEAR(talon::hover_thrust(params), params.mass * params.gravity, 1e-15);
    EXPECT_NEAR(talon::hover_thrust(), talon::MASS * talon::GRAVITY, 1e-15);
}
