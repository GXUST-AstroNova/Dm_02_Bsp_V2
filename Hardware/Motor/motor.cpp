#include "motor.hpp"
#include "irq_guard.hpp"

MotorFeedback Motor::feedback() const noexcept {
    IrqGuard guard;
    return feedback_;
}
