#pragma once
#include "fdcan.h"
#include "can_fd_length.hpp"
#include <span>

namespace can_bus {
using FdReceiver = void (*)(void*, std::span<const uint8_t>) noexcept;
bool subscribeFd(FDCAN_HandleTypeDef& bus, uint32_t id, void* context, FdReceiver receive) noexcept;
void unsubscribeFd(void* context) noexcept;
uint32_t txSlots(FDCAN_HandleTypeDef& bus) noexcept;
// Buffer holds a full FD element. Length is the padded wire length.
bool enqueueFd(FDCAN_HandleTypeDef& bus, uint32_t id, std::span<const uint8_t, 64> buffer,
               uint8_t length) noexcept;
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
