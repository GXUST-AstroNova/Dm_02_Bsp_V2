#include "pwm_buzzer.h"

#include "buzzer_tone.h"

namespace {

/** APB1 定时器时钟：APB1 分频为 1 时等于 PCLK1，否则为 2×PCLK1（H7 规则）。 */
uint32_t Apb1_Timer_Clock_Hz() noexcept {
    const uint32_t pclk1 = HAL_RCC_GetPCLK1Freq();
    const uint32_t hclk = HAL_RCC_GetHCLKFreq();
    return (pclk1 == hclk) ? pclk1 : (2U * pclk1);
}

} // namespace

PwmBuzzer::PwmBuzzer(TIM_HandleTypeDef *tim, const Config &config) noexcept
    : Buzzer(config.min_hz, config.max_hz), tim_(tim), config_(config) {}

int8_t PwmBuzzer::Hardware_Init() noexcept {
    if (tim_ == nullptr || tim_->Instance == nullptr) {
        ++error_count_;
        return -1;
    }
    if (config_.channel != TIM_CHANNEL_2) {
        ++error_count_;
        return -1; // 本实现固定使用 PB15/TIM12_CH2
    }

    const uint32_t timer_hz =
        (config_.timer_clock_hz != 0U) ? config_.timer_clock_hz : Apb1_Timer_Clock_Hz();
    if (timer_hz == 0U) {
        ++error_count_;
        return -1;
    }
    const uint32_t prescaler = static_cast<uint32_t>(tim_->Init.Prescaler) + 1U;
    tick_hz_ = timer_hz / prescaler;
    if (tick_hz_ == 0U) {
        ++error_count_;
        return -1;
    }

    // ARR 预装载：换音时在下一个更新事件生效，减少毛刺（只改 CR1，不动生成代码）。
    tim_->Instance->CR1 |= TIM_CR1_ARPE;
    // CCR 关闭预装载：Stop/静音与换占空比立即生效（最多一次 PWM 周期内的边沿抖动，
    // 对蜂鸣器可忽略），避免“停止后还要等一个低频周期才安静”。
    tim_->Instance->CCMR1 &= ~TIM_CCMR1_OC2PE;

    // 上电静音：先清 CCR 再启动 PWM，输出保持低电平（NPN 关断）。
    __HAL_TIM_SET_COMPARE(tim_, config_.channel, 0U);
    if (HAL_TIM_PWM_Start(tim_, config_.channel) != HAL_OK) {
        ++error_count_;
        return -1;
    }
    return 0;
}

void PwmBuzzer::Apply_Tone(uint16_t freq_hz) noexcept {
    if (tim_ == nullptr || tick_hz_ == 0U) {
        return;
    }
    if (freq_hz == buzzer_note::kRest) {
        __HAL_TIM_SET_COMPARE(tim_, config_.channel, 0U); // 静音：晶体管关断
        return;
    }

    const uint32_t arr = buzzer_tone::Arr_For(freq_hz, tick_hz_);
    if (arr == 0U) {
        ++error_count_;
        return;
    }
    // ARR 先写（ARPE 生效于下一个更新事件），再写 50% 占空比。
    __HAL_TIM_SET_AUTORELOAD(tim_, arr);
    __HAL_TIM_SET_COMPARE(tim_, config_.channel, buzzer_tone::Compare_For(arr));
}
