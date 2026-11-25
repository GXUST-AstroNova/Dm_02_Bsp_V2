#include "pid.hpp"

#include <algorithm>
#include <cmath>

PID::PID(float kp, float ki, float kd, float i_limit, float dt, float output_limit) noexcept
    : kp_(kp), ki_(ki), kd_(kd), i_limit_(i_limit), dt_(dt), output_limit_(output_limit),
      valid_(std::isfinite(kp) && std::isfinite(ki) && std::isfinite(kd) &&
             std::isfinite(dt) && dt > 0.0f && i_limit >= 0.0f && output_limit >= 0.0f) {}

void PID::Reset() noexcept {
    previous_error_ = 0.0f;
    integral_ = 0.0f;
    previous_derivative_ = 0.0f;
    target_ = 0.0f;
    output_ = 0.0f;
    first_step_ = true;
}

PositionalPID::PositionalPID(float kp, float ki, float kd, float i_limit, float dt,
                             float output_limit) noexcept
    : PID(kp, ki, kd, i_limit, dt, output_limit) {}

float PositionalPID::Calculate(float target, float measurement) noexcept {
    if (!valid_ || !std::isfinite(target) || !std::isfinite(measurement)) {
        return output_;
    }
    measurement_ = measurement;
    const float error = target - measurement;
    integral_ = std::clamp(integral_ + error * dt_, -i_limit_, i_limit_);
    const float derivative = first_step_ ? 0.0f : (error - previous_error_) / dt_;
    output_ = std::clamp(kp_ * error + ki_ * integral_ + kd_ * derivative,
                         -output_limit_, output_limit_);

    target_ = target;
    previous_error_ = error;
    first_step_ = false;
    return output_;
}

float PositionalPID::Calculate(float measurement) noexcept {
    return Calculate(target_, measurement);
}

IncrementalPID::IncrementalPID(float kp, float ki, float kd, float i_limit, float dt,
                               float output_limit, float delta_limit) noexcept
    : PID(kp, ki, kd, i_limit, dt, output_limit), delta_limit_(delta_limit) {
    valid_ = valid_ && delta_limit >= 0.0f;
}

float IncrementalPID::Calculate(float target, float measurement) noexcept {
    if (!valid_ || !std::isfinite(target) || !std::isfinite(measurement)) {
        return output_;
    }
    measurement_ = measurement;
    const float error = target - measurement;
    const float delta_error = error - previous_error_;
    const float integral = std::clamp(integral_ + error * dt_, -i_limit_, i_limit_);
    const float derivative = first_step_ ? 0.0f : delta_error / dt_;
    const float raw_delta = kp_ * delta_error + ki_ * (integral - integral_)
                          + kd_ * (derivative - previous_derivative_);
    const float delta = std::clamp(raw_delta, -delta_limit_, delta_limit_);
    output_ = std::clamp(output_ + delta, -output_limit_, output_limit_);

    target_ = target;
    previous_error_ = error;
    integral_ = integral;
    previous_derivative_ = derivative;
    first_step_ = false;
    return output_;
}

float IncrementalPID::Calculate(float measurement) noexcept {
    return Calculate(target_, measurement);
}

IntegralScopePID::IntegralScopePID(float kp, float ki, float kd, float i_limit, float dt,
                                   float integral_scope, float output_limit) noexcept
    : PID(kp, ki, kd, i_limit, dt, output_limit), integral_scope_(integral_scope) {
    valid_ = valid_ && integral_scope >= 0.0f;
}

float IntegralScopePID::Calculate(float target, float measurement) noexcept {
    if (!valid_ || !std::isfinite(target) || !std::isfinite(measurement)) {
        return output_;
    }
    measurement_ = measurement;
    const float error = target - measurement;
    const float integral_step = std::clamp(error * dt_, -integral_scope_, integral_scope_);
    integral_ = std::clamp(integral_ + integral_step, -i_limit_, i_limit_);
    const float derivative = first_step_ ? 0.0f : (error - previous_error_) / dt_;
    output_ = std::clamp(kp_ * error + ki_ * integral_ + kd_ * derivative,
                         -output_limit_, output_limit_);

    target_ = target;
    previous_error_ = error;
    first_step_ = false;
    return output_;
}

float IntegralScopePID::Calculate(float measurement) noexcept {
    return Calculate(target_, measurement);
}
