#pragma once

#include <limits>

/**
 * 令 e = target - measurement，滑模面 s = e + c·integral，输出为
 * k·s + eta·sat(s/phi)，按 ±output_limit 限幅。单参数 Calculate 沿用目标。
 * 非有限输入保持全部状态。Reset 清除目标、积分、前次误差和输出，保留配置。
 */
class SMC {
public:
    // 简式：s = e，逐拍计算，phi > 0。
    SMC(float k, float eta, float phi,
        float output_limit = std::numeric_limits<float>::infinity()) noexcept;

    // PI 滑模面：dt 以秒计，前次误差初始为零。每拍按梯形法积分并按
    // ±integral_limit 限幅。
    // phi <= 0 时采用 sign(s)。有限且非正的 dt 按旧工程约定取 1 ms。
    SMC(float c, float k, float eta, float phi, float output_limit,
        float integral_limit, float dt = 0.001f) noexcept;

    float Calculate(float target, float measurement) noexcept;
    float Calculate(float measurement) noexcept;
    void Reset() noexcept;
    float target_{};
    float output_{};
    float measurement_{};
private:
    float linear_gain_, switching_gain_, boundary_width_, output_limit_;
    bool valid_;
    float integral_gain_{};
    float integral_limit_{};
    float dt_{};
    double integral_{};
    double previous_error_{};
    
};

/**
 * 单轴位置保持。目标与位置使用同一连续角度坐标，速度为该角度单位每秒。
 * 每个目标按静止设定点处理：e = target - position，s = lambda·e - velocity。
 * 返回期望角加速度 -lambda·velocity + k·s + eta·clamp(s/phi, -1, 1)，
 * 按 ±output_limit 限幅。lambda、k 的单位为 1/s，phi 与角速度同单位，
 * eta 与输出同单位。边界层内的切换项随 s 线性变化，无死区。
 * 配置无效时输出零，非有限输入保持目标和上次输出。Reset 将两者清零。
 */
class PositionalSMC {
public:
    PositionalSMC(float lambda, float k, float eta, float phi,
                  float output_limit = std::numeric_limits<float>::infinity()) noexcept;
    float Calculate(float target_position, float position, float velocity) noexcept;
    float Calculate(float position, float velocity) noexcept;
    void Reset() noexcept;
    float target_{};
    float output_{};
    float speed_measurement_{};
    float position_measurement_{};
    private:
    float surface_gain_, linear_gain_, switching_gain_, boundary_width_, output_limit_;
    bool valid_;
    
};
