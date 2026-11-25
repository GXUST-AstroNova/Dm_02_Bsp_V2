#include "imu.hpp"
#include "irq_guard.hpp"

ImuData Imu::Get_Data() const noexcept {
    IrqGuard guard;
    return data_;
}

bool Imu::Is_Online(uint32_t timeout_ms) const noexcept {
    IrqGuard guard;
    return update_count_ && HAL_GetTick() - last_update_tick_ <= timeout_ms;
}

void Imu::markUpdated(const ImuData& sample) noexcept {
    IrqGuard guard;
    data_ = sample;
    last_update_tick_ = HAL_GetTick();
    ++update_count_;
}

void Imu::markFailed() noexcept {
    IrqGuard guard;
    ++refresh_errors_;
}
