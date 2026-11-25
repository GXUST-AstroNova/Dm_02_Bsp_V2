#pragma once

#include <limits>

/**
 * 离散 PID 的公共参数与采样状态。dt 以秒计，i_limit 限制乘 ki 前的累计积分。
 * Calculate(target, measurement) 保存目标并推进一拍，单参数版本沿用已保存的目标。
 * dt 必须为正且有限，增益须有限，限幅允许零至正无穷。无效配置返回零；
 * 目标或测量值非有限时保留上次输出、目标和历史状态。
 */
class PID {
public:
    void Reset() noexcept;

protected:
    static constexpr float unlimited = std::numeric_limits<float>::infinity();

    PID(float kp, float ki, float kd, float i_limit, float dt, float output_limit) noexcept;

    float kp_, ki_, kd_;
    float i_limit_, dt_, output_limit_;
    bool valid_;
    float previous_error_{};
    float integral_{};
    float previous_derivative_{};
    float target_{};
    float output_{};
    bool first_step_{true};
};

/**
 * 位置式 PID。令 e = target − measurement，S = clamp(S_prev + e·dt, ±i_limit)，
 * D = (e − e_prev)/dt。首拍 S 从零累加，D 取零。
 * 每拍返回总输出 clamp(kp·e + ki·S + kd·D, ±output_limit)。
 */
class PositionalPID : public PID {
public:
    PositionalPID(float kp, float ki, float kd, float i_limit, float dt,
                  float output_limit = unlimited) noexcept;
    float Calculate(float target, float measurement) noexcept;
    float Calculate(float measurement) noexcept;
};

/**
 * 增量式 PID。S、D 与位置式相同，首拍历史量为零，P、I 参与计算，D 取零。
 * 原始输出增量为 kp·(e − e_prev) + ki·(S − S_prev) + kd·(D − D_prev)。
 * 先按 ±delta_limit 限制本拍变化，再叠加到上拍实际输出并按 ±output_limit 限幅。
 * 返回本拍实际总输出，被截去的增量当拍舍弃。
 */
class IncrementalPID : public PID {
public:
    IncrementalPID(float kp, float ki, float kd, float i_limit, float dt,
                   float output_limit = unlimited, float delta_limit = unlimited) noexcept;
    float Calculate(float target, float measurement) noexcept;
    float Calculate(float measurement) noexcept;

private:
    float delta_limit_;
};

/**
 * 单次积分限幅的位置式 PID。每拍先取 ΔS = clamp(e·dt, ±integral_scope)，
 * 再取 S = clamp(S_prev + ΔS, ±i_limit)，大误差仍沿其方向积分。
 * 首拍从 S_prev = 0、D = 0 开始，返回 clamp(kp·e + ki·S + kd·D, ±output_limit)。
 */
class IntegralScopePID : public PID {
public:
    IntegralScopePID(float kp, float ki, float kd, float i_limit, float dt,
                     float integral_scope, float output_limit = unlimited) noexcept;
    float Calculate(float target, float measurement) noexcept;
    float Calculate(float measurement) noexcept;

private:
    float integral_scope_;
};
