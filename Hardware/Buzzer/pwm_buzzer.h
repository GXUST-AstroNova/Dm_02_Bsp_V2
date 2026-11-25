//
// TIM12_CH2（PB15）无源蜂鸣器硬件实现
//
// 板载：无源蜂鸣器，NPN 低边驱动，TIM12_CH2 → R20 → Q2 基极，高电平导通；
// 典型谐振频率 4 kHz。MX_TIM12_Init() 已由 main 配置为 PWM1、PSC=274、ARR=249
// （定时器时钟 275 MHz → 计数 1 MHz → 4 kHz）。
//
// 非阻塞：
//   - Hardware_Init() 只推导计数频率、置 ARPE、把 CCR 清 0 并启动 PWM，然后立即返回；
//   - 播放期间音调由 TIM12 硬件方波持续产生，软件不需要逐周期发送；
//   - Play/Update/Stop 见 Buzzer 基类：只做比较和寄存器写，无 HAL_Delay/忙等。
//
// 用法：
//   static PwmBuzzer buzzer(&htim12, PwmBuzzer::Config{});
//   buzzer.Init();
//   buzzer.Play(kSong, count, HAL_GetTick());   // 非阻塞
//   // 主循环：buzzer.Update(HAL_GetTick());
//

#ifndef DM_02_BSP_PWM_BUZZER_H
#define DM_02_BSP_PWM_BUZZER_H

#include <cstdint>

#include "base_buzzer.h"
#include "tim.h"

class PwmBuzzer final : public Buzzer {
  public:
    struct Config {
        uint32_t timer_clock_hz = 0U; ///< 0 = 从 HAL_RCC 自动推导（推荐）
        uint32_t channel = TIM_CHANNEL_2; ///< 仅支持 TIM_CHANNEL_2（PB15）
        uint16_t min_hz = 20U;
        uint16_t max_hz = 10000U;
    };

    PwmBuzzer(TIM_HandleTypeDef *tim, const Config &config) noexcept;

    /** 实际使用的定时器计数频率（Hz），便于调试观察。 */
    uint32_t Get_Tick_Hz() const noexcept { return tick_hz_; }
    uint32_t Get_Error_Count() const noexcept { return error_count_; }

  protected:
    int8_t Hardware_Init() noexcept override;
    void Apply_Tone(uint16_t freq_hz) noexcept override;

  private:
    TIM_HandleTypeDef *tim_ = nullptr;
    Config config_{};
    uint32_t tick_hz_ = 0U;
    uint32_t error_count_ = 0U;
};

#endif // DM_02_BSP_PWM_BUZZER_H
