#pragma once
#include "fdcan.h"

namespace can_bus {
// Call after MX_FDCANx_Init, once per bus used by the application.
bool start(FDCAN_HandleTypeDef* bus) noexcept;
bool Soft_Restart(FDCAN_HandleTypeDef* bus) noexcept;
bool Hard_Restart(FDCAN_HandleTypeDef* bus) noexcept;
}
