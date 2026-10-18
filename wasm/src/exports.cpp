// =============================================================================
// Talon FlightLab WASM bindings
// =============================================================================
// Embind bindings that expose the header-only C++ GNC kernel to JavaScript so
// the FlightLab web app can run a closed-loop SIL live in the browser. The
// Python glue (mission, NLGL guidance, motors, sensors, phase machine) is
// ported to TypeScript; the numerically heavy, safety-critical kernel
// (dynamics RK4, Dryden wind, cascaded PD controller, 15-state MEKF) is the
// same validated C++ used by the native SIL (core/include/talon/).
//
// API shape: one stateful Kernel object with scalar in/out methods (numbers
// marshal cheaply across the embind boundary at 200 Hz). The TypeScript loop
// owns sequencing and rates; the Kernel owns the physics, the estimator, and
// the controller. Method names are snake_case to mirror the C++/Python names.
// =============================================================================

#include <algorithm>
#include <cmath>
#include <vector>

#include <emscripten/bind.h>

#include "talon/constants.hpp"
#include "talon/controller.hpp"
#include "talon/dynamics.hpp"
#include "talon/ekf.hpp"
#include "talon/types.hpp"
#include "talon/wind.hpp"

namespace {

// Component access without relying on Eigen's vector operator[].
double comp(const Eigen::Vector3d& v, int i) {
    return i == 0 ? v.x() : (i == 1 ? v.y() : v.z());
}

}  // namespace

// Stateful kernel handle: truth plant + Dryden wind + cascade controller + EKF.
class Kernel {
public:
    Kernel()
        : params_(),
          controller_(talon::ControlGains::defaults(), params_),
          ekf_() {}

    // ---- Truth plant -------------------------------------------------------
    void set_truth_pos(double x, double y, double z) {
        truth_.position = talon::Vec3(x, y, z);
    }
    void set_truth_vel(double x, double y, double z) {
        truth_.velocity = talon::Vec3(x, y, z);
    }
    void set_truth_quat(double w, double x, double y, double z) {
        truth_.orientation = talon::Quaternion{w, x, y, z}.normalized();
    }
    void set_truth_rate(double x, double y, double z) {
        truth_.angular_rate = talon::Vec3(x, y, z);
    }

    double truth_pos(int i) const { return comp(truth_.position, i); }
    double truth_vel(int i) const { return comp(truth_.velocity, i); }
    double truth_rate(int i) const { return comp(truth_.angular_rate, i); }
    double truth_quat(int i) const {
        const talon::Quaternion& q = truth_.orientation;
        return i == 0 ? q.w : (i == 1 ? q.x : (i == 2 ? q.y : q.z));
    }
    // Body-to-NED direction cosine matrix element (row, col).
    double truth_dcm(int r, int c) const {
        return truth_.orientation.to_dcm()(r, c);
    }
    // Body-to-NED DCM as a flat, row-major 9-element JS array (one call per
    // step instead of nine).
    emscripten::val truth_dcm_flat() const {
        const talon::Vec3 a = truth_.orientation.to_dcm().row(0);
        const talon::Vec3 b = truth_.orientation.to_dcm().row(1);
        const talon::Vec3 c = truth_.orientation.to_dcm().row(2);
        return emscripten::val::array(
            std::vector<double>{a.x(), a.y(), a.z(), b.x(), b.y(), b.z(),
                                c.x(), c.y(), c.z()});
    }

    // ---- Dryden wind -------------------------------------------------------
    // Configure the turbulence generator (sigma per axis, integral scale per
    // axis, step, seed). dt is the plant step so the discretization matches.
    void wind_configure(double su, double sv, double sw, double lu, double lv,
                        double lw, double dt, int seed) {
        wind_ = talon::DrydenWind(su, sv, sw, lu, lv, lw, dt,
                                  static_cast<unsigned int>(seed));
    }
    void wind_steady(double x, double y, double z) {
        wind_steady_ = talon::Vec3(x, y, z);
    }
    void wind_reset() { wind_.reset(); }
    double last_wind(int i) const { return comp(last_wind_, i); }

    // ---- Plant step --------------------------------------------------------
    // Steps the turbulence process (using the horizontal airspeed), advances
    // the truth state one RK4 step with the applied thrust/moment, then applies
    // the ground contact clamp (cap NED z at ground_z, kill downward velocity).
    void plant_step(double thrust, double mx, double my, double mz, double dt,
                    double airspeed, double ground_z) {
        talon::Vec3 wind_ned = wind_steady_ + wind_.step(airspeed);
        last_wind_ = wind_ned;
        truth_ = talon::propagate(truth_, thrust, talon::Vec3(mx, my, mz),
                                  wind_ned, dt, params_);
        if (truth_.position.z() > ground_z) {
            truth_.position.z() = ground_z;
            if (truth_.velocity.z() > 0.0) {
                truth_.velocity.z() = 0.0;
            }
        }
    }

    // ---- EKF ---------------------------------------------------------------
    // Initialize from a position/velocity pair plus a stationary specific
    // force measurement and yaw (mirrors Ekf::initialize_from_accel).
    void ekf_init_pv(double px, double py, double pz, double vx, double vy,
                     double vz) {
        init_pos_ = talon::Vec3(px, py, pz);
        init_vel_ = talon::Vec3(vx, vy, vz);
    }
    void ekf_init_from_accel(double ax, double ay, double az, double yaw) {
        ekf_.initialize_from_accel(init_pos_, init_vel_,
                                   talon::Vec3(ax, ay, az), yaw);
    }
    void ekf_init_level(double yaw) {
        ekf_.initialize_level(init_pos_, init_vel_, yaw);
    }
    void ekf_predict(double ax, double ay, double az, double gx, double gy,
                     double gz, double dt) {
        ekf_.predict(talon::Vec3(ax, ay, az), talon::Vec3(gx, gy, gz), dt);
    }
    // Returns 1 when the GPS fix is accepted, 0 when gated out.
    int ekf_update(double px, double py, double pz, double vx, double vy,
                   double vz) {
        return ekf_.update(talon::Vec3(px, py, pz), talon::Vec3(vx, vy, vz))
                   ? 1
                   : 0;
    }
    double ekf_pos(int i) const { return comp(ekf_.nominal().position, i); }
    double ekf_vel(int i) const { return comp(ekf_.nominal().velocity, i); }
    double ekf_quat(int i) const {
        const talon::Quaternion& q = ekf_.nominal().attitude;
        return i == 0 ? q.w : (i == 1 ? q.x : (i == 2 ? q.y : q.z));
    }
    double ekf_gyro_bias(int i) const {
        return comp(ekf_.nominal().gyro_bias, i);
    }
    double ekf_accel_bias(int i) const {
        return comp(ekf_.nominal().accel_bias, i);
    }
    double ekf_pos_var(int i) const { return ekf_.covariance()(i, i); }
    double ekf_pos_trace() const {
        const talon::Matrix15& P = ekf_.covariance();
        return P(0, 0) + P(1, 1) + P(2, 2);
    }
    // Fault-injection hook: corrupt the estimator's accel-bias estimate
    // (scenario_emergency style EKF divergence).
    void ekf_corrupt_accel_bias(double value) {
        ekf_.mutable_nominal().accel_bias = talon::Vec3(value, 0.0, 0.0);
    }

    // ---- Cascaded PD controller -------------------------------------------
    void controller_reset() { controller_.reset(); }
    // Desired position/velocity (NED) and yaw for the next control step.
    void set_control_target(double px, double py, double pz, double vx, double vy,
                            double vz, double yaw) {
        target_pos_ = talon::Vec3(px, py, pz);
        target_vel_ = talon::Vec3(vx, vy, vz);
        target_yaw_ = yaw;
    }
    // One control step using the EKF nominal state as the estimated state and
    // (gyro - gyro_bias) as the estimated body rates (mirrors
    // SILRunner._estimated_state). Result is read with cmd_thrust()/cmd_moment().
    void control_step(double dt, double gx, double gy, double gz) {
        talon::State est;
        est.position = ekf_.nominal().position;
        est.velocity = ekf_.nominal().velocity;
        est.orientation = ekf_.nominal().attitude;
        est.angular_rate = ekf_.estimated_body_rates(talon::Vec3(gx, gy, gz));
        const talon::ControlCommand cmd =
            controller_.step(est, target_pos_, target_vel_, target_yaw_, dt);
        cmd_thrust_ = std::max(0.0, cmd.collective_thrust);
        cmd_moment_ = cmd.moment;
    }
    double cmd_thrust() const { return cmd_thrust_; }
    double cmd_moment(int i) const { return comp(cmd_moment_, i); }

    // ---- Diagnostics -------------------------------------------------------
    // Hover collective thrust (N): mass * gravity, for the ground-hold check.
    double hover_thrust() const { return talon::hover_thrust(params_); }

private:
    talon::VehicleParams params_;
    talon::State truth_;
    talon::CascadedController controller_;
    talon::Ekf ekf_;
    talon::DrydenWind wind_;
    talon::Vec3 wind_steady_ = talon::Vec3::Zero();
    talon::Vec3 last_wind_ = talon::Vec3::Zero();
    talon::Vec3 init_pos_ = talon::Vec3::Zero();
    talon::Vec3 init_vel_ = talon::Vec3::Zero();
    talon::Vec3 target_pos_ = talon::Vec3::Zero();
    talon::Vec3 target_vel_ = talon::Vec3::Zero();
    double target_yaw_ = 0.0;
    double cmd_thrust_ = 0.0;
    talon::Vec3 cmd_moment_ = talon::Vec3::Zero();
};

EMSCRIPTEN_BINDINGS(talon_kernel) {
    emscripten::class_<Kernel>("Kernel")
        .constructor<>()
        // truth
        .function("set_truth_pos", &Kernel::set_truth_pos)
        .function("set_truth_vel", &Kernel::set_truth_vel)
        .function("set_truth_quat", &Kernel::set_truth_quat)
        .function("set_truth_rate", &Kernel::set_truth_rate)
        .function("truth_pos", &Kernel::truth_pos)
        .function("truth_vel", &Kernel::truth_vel)
        .function("truth_rate", &Kernel::truth_rate)
        .function("truth_quat", &Kernel::truth_quat)
        .function("truth_dcm", &Kernel::truth_dcm)
        .function("truth_dcm_flat", &Kernel::truth_dcm_flat)
        // wind
        .function("wind_configure", &Kernel::wind_configure)
        .function("wind_steady", &Kernel::wind_steady)
        .function("wind_reset", &Kernel::wind_reset)
        .function("last_wind", &Kernel::last_wind)
        // plant
        .function("plant_step", &Kernel::plant_step)
        // ekf
        .function("ekf_init_pv", &Kernel::ekf_init_pv)
        .function("ekf_init_from_accel", &Kernel::ekf_init_from_accel)
        .function("ekf_init_level", &Kernel::ekf_init_level)
        .function("ekf_predict", &Kernel::ekf_predict)
        .function("ekf_update", &Kernel::ekf_update)
        .function("ekf_pos", &Kernel::ekf_pos)
        .function("ekf_vel", &Kernel::ekf_vel)
        .function("ekf_quat", &Kernel::ekf_quat)
        .function("ekf_gyro_bias", &Kernel::ekf_gyro_bias)
        .function("ekf_accel_bias", &Kernel::ekf_accel_bias)
        .function("ekf_pos_var", &Kernel::ekf_pos_var)
        .function("ekf_pos_trace", &Kernel::ekf_pos_trace)
        .function("ekf_corrupt_accel_bias", &Kernel::ekf_corrupt_accel_bias)
        // controller
        .function("controller_reset", &Kernel::controller_reset)
        .function("set_control_target", &Kernel::set_control_target)
        .function("control_step", &Kernel::control_step)
        .function("cmd_thrust", &Kernel::cmd_thrust)
        .function("cmd_moment", &Kernel::cmd_moment)
        // diagnostics
        .function("hover_thrust", &Kernel::hover_thrust);
}
