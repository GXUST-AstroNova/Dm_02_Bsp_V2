#include "smc.hpp"

#include <algorithm>
#include <cmath>

SMC::SMC(float k, float eta, float phi, float output_limit) noexcept
    : linear_gain_(k), switching_gain_(eta), boundary_width_(phi),
      output_limit_(std::min(output_limit, std::numeric_limits<float>::max())),
      valid_(std::isfinite(k) && k >= 0.0f && std::isfinite(eta) && eta > 0.0f &&
             std::isfinite(phi) && phi > 0.0f && output_limit >= 0.0f) {}

SMC::SMC(float c, float k, float eta, float phi, float output_limit,
         float integral_limit, float dt) noexcept
    : linear_gain_(k), switching_gain_(eta), boundary_width_(phi),
      output_limit_(std::min(output_limit, std::numeric_limits<float>::max())),
      valid_(std::isfinite(c) && std::isfinite(k) && std::isfinite(eta) &&
             std::isfinite(phi) && output_limit >= 0.0f && integral_limit >= 0.0f &&
             std::isfinite(dt)),
      integral_gain_(c), integral_limit_(integral_limit),
      dt_(dt > 0.0f ? dt : 0.001f) {}

float SMC::Calculate(float target, float measurement) noexcept {
    if (!valid_ || !std::isfinite(target) || !std::isfinite(measurement)) {
        return output_;
    }

    // 使用较宽的中间量，避免两个有限 float 的差值在限幅前溢出。
    const double error = static_cast<double>(target) - measurement;
    const double last_error = previous_error_;
    previous_error_ = error;
    target_ = target;

    const double integral_limit = integral_limit_;
    integral_ = std::clamp(integral_ + 0.5 * (error + last_error) * dt_,
                           -integral_limit, integral_limit);
    const double surface = error + integral_gain_ * integral_;
    const double switching = boundary_width_ > 0.0f
        ? std::clamp(surface / boundary_width_, -1.0, 1.0)
        : (surface > 0.0) - (surface < 0.0);
    const double command = linear_gain_ * surface + switching_gain_ * switching;
    const double limit = output_limit_;
    output_ = static_cast<float>(std::clamp(command, -limit, limit));
    return output_;
}

float SMC::Calculate(float measurement) noexcept {
    return Calculate(target_, measurement);
}

void SMC::Reset() noexcept {
    previous_error_ = 0.0;
    integral_ = 0.0;
    target_ = 0.0f;
    output_ = 0.0f;
}

PositionalSMC::PositionalSMC(float lambda, float k, float eta, float phi,
                             float output_limit) noexcept
    : surface_gain_(lambda), linear_gain_(k), switching_gain_(eta), boundary_width_(phi),
      output_limit_(std::min(output_limit, std::numeric_limits<float>::max())),
      valid_(std::isfinite(lambda) && lambda > 0.0f && std::isfinite(k) && k >= 0.0f &&
             std::isfinite(eta) && eta >= 0.0f && std::isfinite(phi) && phi > 0.0f &&
             output_limit >= 0.0f) {}

float PositionalSMC::Calculate(float target_position, float position, float velocity) noexcept {
    if (!valid_ || !std::isfinite(target_position) ||
        !std::isfinite(position) || !std::isfinite(velocity)) {
        return output_;
    }

    const double error = static_cast<double>(target_position) - position;
    const double rate = velocity;
    const double surface = surface_gain_ * error - rate;
    const double switching = std::clamp(surface / boundary_width_, -1.0, 1.0);
    const double acceleration = -surface_gain_ * rate + linear_gain_ * surface
                                + switching_gain_ * switching;
    const double limit = output_limit_;
    output_ = static_cast<float>(std::clamp(acceleration, -limit, limit));
    target_ = target_position;
    return output_;
}

float PositionalSMC::Calculate(float position, float velocity) noexcept {
    return Calculate(target_, position, velocity);
}

void PositionalSMC::Reset() noexcept {
    target_ = 0.0f;
    output_ = 0.0f;
}
