#include "trajectory_smc.hpp"

#include <algorithm>
#include <cmath>

TrajectorySMC::TrajectorySMC(float lambda, float k, float eta, float phi,
                             float output_limit) noexcept
    : PositionalSMC(lambda, k, eta, phi),
      output_limit_(std::min(output_limit, std::numeric_limits<float>::max())) {
    // 基类配置无效时反馈为零，此时也禁用前馈。
    if (!(std::isfinite(lambda) && lambda > 0.0f && std::isfinite(k) && k >= 0.0f &&
          std::isfinite(eta) && eta >= 0.0f && std::isfinite(phi) && phi > 0.0f &&
          output_limit >= 0.0f)) {
        output_limit_ = 0.0f;
    }
}

float TrajectorySMC::Calculate(float target_position, float target_velocity,
                               float target_acceleration, float position, float velocity) noexcept {
    if (!std::isfinite(target_position) || !std::isfinite(target_velocity) ||
        !std::isfinite(target_acceleration) || !std::isfinite(position) ||
        !std::isfinite(velocity)) {
        return output_;
    }

    const double relative_velocity = static_cast<double>(velocity) - target_velocity;
    if (std::abs(relative_velocity) > std::numeric_limits<float>::max()) {
        return output_;
    }
    const float feedback = PositionalSMC::Calculate(target_position, position,
                                                     static_cast<float>(relative_velocity));
    const double command = static_cast<double>(feedback) + target_acceleration;
    const double limit = output_limit_;
    output_ = static_cast<float>(std::clamp(command, -limit, limit));
    return output_;
}

void TrajectorySMC::Reset() noexcept {
    PositionalSMC::Reset();
    output_ = 0.0f;
}
