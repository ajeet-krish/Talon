#pragma once

#include <array>
#include <cmath>
#include <random>

#include <Eigen/Dense>

#include "talon/constants.hpp"

namespace talon {

// Simplified Dryden wind turbulence: per-axis first-order Gauss-Markov process
// (the continuous-time shaping filter from MIL-F-8785C / Beard & McLain,
// discretized exactly over one step):
//
//   w[k+1] = exp(-Va*dt/L) * w[k] + sigma * sqrt(1 - exp(-2*Va*dt/L)) * N(0, 1)
//
// The three axes (u, v, w) are independent with their own sigma and integral
// scale L. Phase 1 treats the turbulence axes as aligned with NED (documented
// simplification; a body-axis version can replace it in a later phase).
// Reproducible: std::mt19937 with a fixed seed. At Va = 0 the process freezes
// (no turbulence evolution, matching the physical intuition that turbulence is
// swept past a moving vehicle).
class DrydenWind {
public:
    DrydenWind(double sigma_u = 2.0, double sigma_v = 2.0, double sigma_w = 1.0,
               double L_u = 200.0, double L_v = 200.0, double L_w = 50.0,
               double dt = DT, unsigned int seed = 42)
        : sigma_{sigma_u, sigma_v, sigma_w},
          integral_scale_{L_u, L_v, L_w},
          dt_(dt),
          seed_(seed) {
        reset();
    }

    // Advance the turbulence process one step, return current wind [u, v, w]
    // in NED (m/s).
    Eigen::Vector3d step(double airspeed) {
        const double va = std::max(airspeed, 0.0);
        for (int i = 0; i < 3; ++i) {
            const double alpha = std::exp(-va * dt_ / integral_scale_[i]);
            const double noise = sigma_[i] * std::sqrt(1.0 - alpha * alpha);
            wind_[i] = alpha * wind_[i] + noise * normal_(rng_);
        }
        return wind_;
    }

    // Return to the initial condition (zero wind) and re-seed the generator so
    // the exact same sequence is produced again. The distribution is reset as
    // well: std::normal_distribution caches a Box-Muller deviate internally,
    // and that cache does not die when the engine is re-seeded.
    void reset() {
        rng_.seed(seed_);
        normal_ = std::normal_distribution<double>(0.0, 1.0);
        wind_.setZero();
    }

    Eigen::Vector3d wind() const { return wind_; }

private:
    std::array<double, 3> sigma_;
    std::array<double, 3> integral_scale_;
    double dt_;
    unsigned int seed_;
    Eigen::Vector3d wind_ = Eigen::Vector3d::Zero();
    std::mt19937 rng_;
    std::normal_distribution<double> normal_{0.0, 1.0};
};

}  // namespace talon
