#include <cmath>
#include <vector>

#include <gtest/gtest.h>

#include "talon/constants.hpp"
#include "talon/controller.hpp"
#include "talon/dynamics.hpp"
#include "talon/types.hpp"

using talon::CascadedController;
using talon::ControlCommand;
using talon::ControlGains;
using talon::Quaternion;
using talon::State;
using talon::Vec3;
using talon::VehicleParams;

namespace {

constexpr double kDeg = 0.017453292519943295;

double roll_from_state(const State& s) {
    const Eigen::Matrix3d R = s.orientation.to_dcm();
    return std::atan2(R(2, 1), R(2, 2));
}

double pitch_from_state(const State& s) {
    const Eigen::Matrix3d R = s.orientation.to_dcm();
    return std::asin(-R(2, 0));
}

double yaw_from_state(const State& s) {
    const Eigen::Matrix3d R = s.orientation.to_dcm();
    return std::atan2(R(1, 0), R(0, 0));
}

// Hamilton quaternion for a rotation of `angle` about `axis` (unit).
Quaternion quat_from_axis_angle(double angle, const Vec3& axis) {
    const double half = 0.5 * angle;
    const double s = std::sin(half);
    return Quaternion{std::cos(half), s * axis.x(), s * axis.y(), s * axis.z()};
}

// Closed-loop run with instantaneous motors (the commanded wrench reaches the
// plant directly; motor lag is covered by the Python scenario and pytest).
// Control steps every DT (200 Hz) to keep the C++ acceptance tests fast and
// free of ZOH discretization effects.
struct ClosedLoopRun {
    State final_state;
    std::vector<Vec3> positions;
    std::vector<double> rolls;
};

ClosedLoopRun run_closed_loop(const State& initial,
                              const ControlGains& gains,
                              const VehicleParams& params,
                              const Vec3& pos_desired,
                              const Vec3& vel_desired,
                              double yaw_desired,
                              double duration,
                              bool lock_position = false,
                              const Vec3& wind_ned = Vec3::Zero()) {
    ClosedLoopRun run;
    CascadedController controller(gains, params);
    State state = initial;
    const int steps = static_cast<int>(duration / talon::DT);
    run.positions.reserve(static_cast<size_t>(steps));
    run.rolls.reserve(static_cast<size_t>(steps));
    for (int i = 0; i < steps; ++i) {
        const ControlCommand cmd = controller.step(
            state, pos_desired, vel_desired, yaw_desired, talon::DT);
        state = talon::propagate(state, cmd.collective_thrust, cmd.moment,
                                 wind_ned, talon::DT, params);
        if (lock_position) {
            // Attitude-loop tests: pin the translation so the position PD
            // holds a constant lateral acceleration command while the
            // attitude evolves (pure inner-loop step response).
            state.position = Vec3::Zero();
            state.velocity = Vec3::Zero();
        }
        run.positions.push_back(state.position);
        run.rolls.push_back(roll_from_state(state));
    }
    run.final_state = state;
    return run;
}

}  // namespace

TEST(Controller, DefaultsPositive) {
    const ControlGains gains = ControlGains::defaults();
    for (int i = 0; i < 3; ++i) {
        EXPECT_GT(gains.kp_pos[i], 0.0);
        EXPECT_GT(gains.kd_pos[i], 0.0);
        EXPECT_GT(gains.kp_att[i], 0.0);
        EXPECT_GT(gains.kd_att[i], 0.0);
    }
}

TEST(Controller, HoverEquilibriumThrustAndZeroMoment) {
    // accel_to_attitude consistency: at zero error the gravity-compensated
    // command is a_des = [0, 0, g], so the vehicle must be commanded level
    // (z_b = [0, 0, 1]) with T = m*g and zero moment.
    const VehicleParams params;
    const ControlGains gains = ControlGains::defaults();
    CascadedController controller(gains, params);

    const State hover;  // origin, rest, identity orientation
    const ControlCommand cmd =
        controller.step(hover, Vec3::Zero(), Vec3::Zero(), 0.0, talon::DT);

    EXPECT_NEAR(cmd.collective_thrust, params.mass * params.gravity, 1e-12);
    EXPECT_LT(cmd.moment.norm(), 1e-12);
    EXPECT_LT((controller.last_a_des() - Vec3(0.0, 0.0, params.gravity)).norm(),
              1e-12);
    EXPECT_LT(controller.last_attitude_error().norm(), 1e-12);
}

TEST(Controller, AttitudeStepRollMomentSign) {
    // Command a 10 deg roll via a lateral position offset:
    // a_cmd_y = kp_pos.y * d, desired roll = atan(a_cmd_y / g) = 10 deg.
    const VehicleParams params;
    const ControlGains gains = ControlGains::defaults();
    const double d = params.gravity * std::tan(10.0 * kDeg) / gains.kp_pos.y();
    const Vec3 pos_desired(0.0, d, 0.0);

    CascadedController controller(gains, params);

    // Level vehicle, commanded right-wing-down: positive roll error must
    // produce a positive (roll-right) moment about body x, which rolls the
    // vehicle toward the commanded attitude. This pins the attitude-error
    // sign convention.
    State state;
    const ControlCommand cmd =
        controller.step(state, pos_desired, Vec3::Zero(), 0.0, talon::DT);
    EXPECT_GT(controller.last_attitude_error().x(), 0.0);
    EXPECT_GT(cmd.moment.x(), 0.0);

    // Vehicle already rolled +10 deg with the same command: near-zero error
    // and near-zero moment (equilibrium attitude).
    const double phi = 10.0 * kDeg;
    state.orientation = quat_from_axis_angle(phi, Vec3::UnitX());
    const ControlCommand cmd2 =
        controller.step(state, pos_desired, Vec3::Zero(), 0.0, talon::DT);
    EXPECT_LT(controller.last_attitude_error().norm(), 1e-2);
    EXPECT_LT(cmd2.moment.norm(), 0.2);
}

TEST(Controller, ClosedLoopHoverTenMeters) {
    // Hover 60 s at 10 m altitude. Instantaneous motors; acceptance after a
    // 10 s settling window: position error < 0.05 m per axis, velocity
    // < 0.05 m/s, attitude near level.
    const VehicleParams params;
    const ControlGains gains = ControlGains::defaults();
    const Vec3 pos_desired(0.0, 0.0, -10.0);

    const ClosedLoopRun run = run_closed_loop(
        State{}, gains, params, pos_desired, Vec3::Zero(), 0.0, 60.0);

    const State& s = run.final_state;
    const Vec3 pos_error = pos_desired - s.position;
    EXPECT_LT(std::abs(pos_error.x()), 0.05);
    EXPECT_LT(std::abs(pos_error.y()), 0.05);
    EXPECT_LT(std::abs(pos_error.z()), 0.05);
    EXPECT_LT(s.velocity.norm(), 0.05);
    EXPECT_LT(std::abs(roll_from_state(s)), 0.02);
    EXPECT_LT(std::abs(pitch_from_state(s)), 0.02);
    EXPECT_LT(std::abs(yaw_from_state(s)), 0.02);

    // Steady-state window: last 2 s of the run stay inside the envelope.
    const size_t window = static_cast<size_t>(2.0 / talon::DT);
    for (size_t i = run.positions.size() - window; i < run.positions.size();
         ++i) {
        EXPECT_LT(std::abs(pos_desired.x() - run.positions[i].x()), 0.05);
        EXPECT_LT(std::abs(pos_desired.y() - run.positions[i].y()), 0.05);
        EXPECT_LT(std::abs(pos_desired.z() - run.positions[i].z()), 0.05);
    }
}

TEST(Controller, ClosedLoopPositionStep) {
    // 1 m north step from hover at the origin. Metrics: rise time (0 to 90%
    // of 1 m) < 2 s, overshoot < 10% (peak x < 1.1 m), steady-state error
    // < 0.05 m after 10 s.
    const VehicleParams params;
    const ControlGains gains = ControlGains::defaults();

    const ClosedLoopRun run = run_closed_loop(
        State{}, gains, params, Vec3(1.0, 0.0, 0.0), Vec3::Zero(), 0.0, 12.0);

    double rise_time = -1.0;
    double peak_x = -1e9;
    for (size_t i = 0; i < run.positions.size(); ++i) {
        const double x = run.positions[i].x();
        if (rise_time < 0.0 && x >= 0.9) {
            rise_time = static_cast<double>(i) * talon::DT;
        }
        if (x > peak_x) {
            peak_x = x;
        }
    }
    EXPECT_GT(rise_time, 0.0);
    EXPECT_LT(rise_time, 2.0);
    EXPECT_LT(peak_x, 1.1);

    const State& s = run.final_state;
    EXPECT_LT(std::abs(s.position.x() - 1.0), 0.05);
    EXPECT_LT(std::abs(s.position.y()), 0.05);
    EXPECT_LT(std::abs(s.position.z()), 0.05);
    EXPECT_LT(s.velocity.norm(), 0.05);
}

TEST(Controller, WindRejectionZeroSSError) {
    // 60 s hover at 10 m under a constant 3 m/s North wind. The position
    // integral (ki_pos = 0.3, tau_i = 13.3 s) must null the drag-bias
    // steady-state error that pure PD leaves behind: |x error| < 0.05 m
    // over the last 10 s of the run.
    const VehicleParams params;
    const ControlGains gains = ControlGains::defaults();
    const Vec3 pos_desired(0.0, 0.0, -10.0);
    const Vec3 wind_ned(3.0, 0.0, 0.0);

    const ClosedLoopRun run = run_closed_loop(
        State{}, gains, params, pos_desired, Vec3::Zero(), 0.0, 60.0,
        /*lock_position=*/false, wind_ned);

    const size_t window = static_cast<size_t>(10.0 / talon::DT);
    for (size_t i = run.positions.size() - window; i < run.positions.size();
         ++i) {
        EXPECT_LT(std::abs(pos_desired.x() - run.positions[i].x()), 0.05);
    }
}

TEST(Controller, AntiWindupBoundedIntegral) {
    // 20 m north position step saturates thrust transiently; the
    // back-calculation anti-windup must keep the position integrator from
    // winding up unboundedly: ||integral|| < 5.0 m*s and the vehicle lands
    // within 1 m of the 20 m target after 30 s. Driven with a visible
    // controller so integral_position() can be read at the end.
    const VehicleParams params;
    const ControlGains gains = ControlGains::defaults();
    const Vec3 pos_desired(20.0, 0.0, 0.0);

    CascadedController controller(gains, params);
    State state;
    const int steps = static_cast<int>(30.0 / talon::DT);
    for (int i = 0; i < steps; ++i) {
        const ControlCommand cmd = controller.step(
            state, pos_desired, Vec3::Zero(), 0.0, talon::DT);
        state = talon::propagate(state, cmd.collective_thrust, cmd.moment,
                                 Vec3::Zero(), talon::DT, params);
    }
    EXPECT_LT(controller.integral_position().norm(), 5.0);
    EXPECT_LT(std::abs(state.position.x() - 20.0), 1.0);
}

TEST(Controller, ClosedLoopPositionStepRegression) {
    // Regression pin for the Sprint 2.1 integral addition: the 1 m north
    // step acceptance envelope (rise < 2 s, overshoot < 10%, SSE < 0.05 m)
    // must hold with the defaults() gains, which now include ki_pos = 0.3.
    const VehicleParams params;
    const ControlGains gains = ControlGains::defaults();

    const ClosedLoopRun run = run_closed_loop(
        State{}, gains, params, Vec3(1.0, 0.0, 0.0), Vec3::Zero(), 0.0, 12.0);

    double rise_time = -1.0;
    double peak_x = -1e9;
    for (size_t i = 0; i < run.positions.size(); ++i) {
        const double x = run.positions[i].x();
        if (rise_time < 0.0 && x >= 0.9) {
            rise_time = static_cast<double>(i) * talon::DT;
        }
        if (x > peak_x) {
            peak_x = x;
        }
    }
    EXPECT_GT(rise_time, 0.0);
    EXPECT_LT(rise_time, 2.0);
    EXPECT_LT(peak_x, 1.1);

    const State& s = run.final_state;
    EXPECT_LT(std::abs(s.position.x() - 1.0), 0.05);
    EXPECT_LT(std::abs(s.position.y()), 0.05);
    EXPECT_LT(std::abs(s.position.z()), 0.05);
    EXPECT_LT(s.velocity.norm(), 0.05);
}

TEST(Controller, ClosedLoopAttitudeStepRollTenDegrees) {
    // Command a 10 deg roll with the position pinned at the origin (so the
    // outer loop holds a constant lateral acceleration command and the test
    // measures the inner-loop step response). Rise time to 90% < 0.3 s,
    // overshoot < 15%, steady state at the commanded 10 deg.
    const VehicleParams params;
    ControlGains gains = ControlGains::defaults();
    // Sprint 2.1: pin the outer-loop integral to zero here. The pinned
    // position makes the position error constant, which winds the (new)
    // position integrator and turns the constant lateral acceleration
    // command into a ramp; the inner-loop step response is unaffected by
    // ki_pos, so zeroing it restores this test's constant-command premise.
    gains.ki_pos = Vec3::Zero();
    const double d = params.gravity * std::tan(10.0 * kDeg) / gains.kp_pos.y();

    const ClosedLoopRun run = run_closed_loop(
        State{}, gains, params, Vec3(0.0, d, 0.0), Vec3::Zero(), 0.0, 2.0,
        /*lock_position=*/true);

    const double target = 10.0 * kDeg;
    double rise_time = -1.0;
    double peak_roll = -1e9;
    for (size_t i = 0; i < run.rolls.size(); ++i) {
        if (rise_time < 0.0 && run.rolls[i] >= 0.9 * target) {
            rise_time = static_cast<double>(i) * talon::DT;
        }
        if (run.rolls[i] > peak_roll) {
            peak_roll = run.rolls[i];
        }
    }
    EXPECT_GT(rise_time, 0.0);
    EXPECT_LT(rise_time, 0.3);
    EXPECT_LT(peak_roll, 1.15 * target);
    EXPECT_NEAR(roll_from_state(run.final_state), target, 0.5 * kDeg);
}

TEST(Controller, ClosedLoopYawTrackingThirtyDegrees) {
    // Hover in place while commanding a 30 deg yaw heading. Steady-state yaw
    // error must fall below 2 deg.
    const VehicleParams params;
    const ControlGains gains = ControlGains::defaults();
    const double target = 30.0 * kDeg;

    const ClosedLoopRun run = run_closed_loop(
        State{}, gains, params, Vec3::Zero(), Vec3::Zero(), target, 5.0);

    const State& s = run.final_state;
    EXPECT_LT(std::abs(yaw_from_state(s) - target), 2.0 * kDeg);
    // Translation stayed pinned near the origin (yaw-only maneuver).
    EXPECT_LT(s.position.norm(), 0.02);
    EXPECT_LT(s.velocity.norm(), 0.02);
}

TEST(Controller, ResetProducesIdenticalOutput) {
    const VehicleParams params;
    const ControlGains gains = ControlGains::defaults();
    CascadedController controller(gains, params);

    State state;
    state.position = Vec3(0.3, -0.2, -1.7);
    state.velocity = Vec3(0.4, 0.1, -0.3);
    state.angular_rate = Vec3(0.5, -0.2, 0.1);
    state.orientation = quat_from_axis_angle(0.3, Vec3::UnitY());
    const Vec3 pos_des(1.0, 0.5, -2.0);
    const Vec3 vel_des(0.1, 0.0, 0.0);
    const double yaw_des = 0.4;

    const ControlCommand first =
        controller.step(state, pos_des, vel_des, yaw_des, talon::DT);
    const Vec3 a_des_first = controller.last_a_des();
    const Vec3 e_att_first = controller.last_attitude_error();

    // Run off course with different inputs and a non-zero desired rate to
    // load internal state, then reset.
    controller.step(State{}, Vec3(5.0, 5.0, 5.0), Vec3(1.0, 1.0, 1.0), -1.0,
                    talon::DT);
    controller.set_omega_desired(Vec3(1.0, 2.0, 3.0));
    controller.reset();

    const ControlCommand second =
        controller.step(state, pos_des, vel_des, yaw_des, talon::DT);
    EXPECT_EQ(first.collective_thrust, second.collective_thrust);
    EXPECT_EQ(first.moment, second.moment);
    EXPECT_EQ(controller.last_a_des(), a_des_first);
    EXPECT_EQ(controller.last_attitude_error(), e_att_first);
}

TEST(Controller, ThrustSaturationAndNoNaN) {
    // An insane position command must saturate thrust and stay finite.
    const VehicleParams params;
    const ControlGains gains = ControlGains::defaults();
    CascadedController controller(gains, params);

    State state;
    const ControlCommand cmd = controller.step(
        state, Vec3(1000.0, 1000.0, -1000.0), Vec3::Zero(), 0.0, talon::DT);
    EXPECT_LE(cmd.collective_thrust, controller.thrust_max() + 1e-12);
    EXPECT_GE(cmd.collective_thrust, 0.0);
    EXPECT_TRUE(cmd.moment.allFinite());

    // Exactly g/kp above the target at rest: the gravity-compensated command
    // crosses zero (free-fall degenerate case); must not produce NaN.
    state.position = Vec3(0.0, 0.0, -params.gravity / gains.kp_pos.z());
    const ControlCommand cmd2 = controller.step(
        state, Vec3::Zero(), Vec3::Zero(), 0.0, talon::DT);
    EXPECT_TRUE(std::isfinite(cmd2.collective_thrust));
    EXPECT_TRUE(cmd2.moment.allFinite());
    EXPECT_TRUE(controller.last_a_des().allFinite());
}
