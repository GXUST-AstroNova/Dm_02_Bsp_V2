#pragma once

#include "main.h"

// Preserve the caller's PRIMASK, including when entered from an ISR.
class IrqGuard final {
public:
    IrqGuard() noexcept : previous_(__get_PRIMASK()) { __disable_irq(); }
    ~IrqGuard() { __set_PRIMASK(previous_); }
    IrqGuard(const IrqGuard&) = delete;
    IrqGuard& operator=(const IrqGuard&) = delete;
private:
    uint32_t previous_;
};
