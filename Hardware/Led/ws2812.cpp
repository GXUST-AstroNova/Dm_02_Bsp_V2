#include "ws2812.h"

Ws2812::Ws2812(SPI_HandleTypeDef *spi, const Config &config) noexcept
    : spi_(spi), config_(config), codec_(config.spi_clock_hz) {}

bool Ws2812::Start_Transfer_() noexcept {
    const uint16_t len =
        codec_.Encode(&color_, 1U, config_.order, config_.brightness, tx_buf_,
                      static_cast<uint16_t>(sizeof(tx_buf_)));
    if (len == 0U) {
        ++error_count_;
        return false;
    }
    if (spi_ == nullptr || HAL_SPI_Transmit_IT(spi_, tx_buf_, len) != HAL_OK) {
        ++error_count_;
        return false;
    }
    ++show_count_;
    return true;
}

int8_t Ws2812::Init() noexcept {
    if (ready_) {
        return 0;
    }
    if (spi_ == nullptr || spi_->Instance != SPI6) {
        ++error_count_;
        return -1;
    }
    if (!codec_.Ok()) {
        ++error_count_;
        return -1;
    }
    if (HAL_SPI_GetState(spi_) != HAL_SPI_STATE_READY) {
        ++error_count_;
        return -1;
    }

    // SPI6 中断：工程原本未启用该中断，stm32h7xx_it.c 中没有 SPI6_IRQHandler，
    // 由本驱动提供（见文件末尾）。优先级 7 低于底盘控制 TIM13（6）与 CAN（5）。
    HAL_NVIC_SetPriority(SPI6_IRQn, 7U, 0U);
    HAL_NVIC_EnableIRQ(SPI6_IRQn);

    ready_ = true;
    color_ = LedColor{0U, 0U, 0U};
    if (!Start_Transfer_()) {
        ready_ = false;
        return -1;
    }
    has_shown_ = true;
    last_start_ms_ = HAL_GetTick();
    return 0;
}

bool Ws2812::Set_Color(LedColor color) noexcept {
    color_ = color;
    return ready_;
}

bool Ws2812::Show(uint32_t now_ms) noexcept {
    if (!ready_ || spi_ == nullptr) {
        return false;
    }
    if (Busy()) {
        return false;
    }
    const HAL_SPI_StateTypeDef state = HAL_SPI_GetState(spi_);
    if (state != HAL_SPI_STATE_READY) {
        ++error_count_;
        (void)Recover(); // 错误态自动软复位；本次返回 false，下次 Show 重试
        return false;
    }
    if (has_shown_ &&
        static_cast<uint32_t>(now_ms - last_start_ms_) <
            (config_.min_interval_ms != 0U ? config_.min_interval_ms : 1U)) {
        return false;
    }
    if (!Start_Transfer_()) {
        return false;
    }
    has_shown_ = true;
    last_start_ms_ = now_ms;
    return true;
}

bool Ws2812::Recover() noexcept {
    if (spi_ == nullptr || spi_->Instance != SPI6 || Busy()) {
        return false;
    }
    // 无传输在途时的软复位：DeInit 只关外设/GPIO，Init 重新配置，均为寄存器操作，
    // 不等待传输完成，因此不阻塞调用方。
    if (HAL_SPI_DeInit(spi_) != HAL_OK) {
        ++error_count_;
        ready_ = false;
        return false;
    }
    if (HAL_SPI_Init(spi_) != HAL_OK) {
        ++error_count_;
        ready_ = false;
        return false;
    }
    ready_ = true;
    has_shown_ = false;
    return true;
}

bool Ws2812::Busy() const noexcept {
    return spi_ != nullptr && HAL_SPI_GetState(spi_) == HAL_SPI_STATE_BUSY_TX;
}

// SPI6 全局中断：把中断转交 HAL，完成状态由 HAL 更新；不使用全局回调。
extern "C" void SPI6_IRQHandler(void) {
    HAL_SPI_IRQHandler(&hspi6);
}
