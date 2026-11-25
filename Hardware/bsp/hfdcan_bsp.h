#pragma once
#include "fdcan.h"

namespace can_bus {
// Call after MX_FDCANx_Init, once per bus used by the application.
// Enables the receive FIFO notification and Bus_Off detection. A Bus_Off event
// triggers Soft_Restart from the interrupt handler, which re-enters the bus-off
// recovery sequence without the application having to poll.
bool start(FDCAN_HandleTypeDef* bus) noexcept;
// Abort pending Tx requests, stop and restart the controller. Keeps the filter
// and bit timing, so it is the cheap way back from Bus_Off.
bool Soft_Restart(FDCAN_HandleTypeDef* bus) noexcept;
// DeInit/Init the peripheral and restart it, reconfiguring the filter. Escalate
// to this when Soft_Restart fails.
bool Hard_Restart(FDCAN_HandleTypeDef* bus) noexcept;
}
