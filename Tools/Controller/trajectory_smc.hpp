#pragma once

#include "smc.hpp"

// 单轴轨迹跟踪。每拍传入同一时刻的参考值，位置使用连续角度。
// 若角度单位为 rad，则角速度为 rad/s，角加速度与返回值为 rad/s²。
class TrajectorySMC : private PositionalSMC {
public:
    TrajectorySMC(float lambda, float k, float eta, float phi,
                  float output_limit = std::numeric_limits<float>::infinity()) noexcept;

    // 无效配置输出零。非有限输入或超出 float 范围的相对速度保持上次输出。
    float Calculate(float target_position, float target_velocity, float target_acceleration,
                    float position, float velocity) noexcept;
    void Reset() noexcept;

private:
    float output_limit_;
    float output_{};
};
