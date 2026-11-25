#pragma once

#include "fdcan.h"
#include "motor.hpp"
#include <array>

namespace motor_can {
using Payload = std::array<uint8_t, 8>;

inline bool usesFdBrs(const FDCAN_HandleTypeDef* bus) noexcept {
    return bus->Init.FrameFormat == FDCAN_FRAME_FD_BRS;
}

inline bool enqueue(FDCAN_HandleTypeDef* bus, uint16_t id, const Payload& data,
                    bool fd_brs = false) noexcept {
    FDCAN_TxHeaderTypeDef header{};
    header.Identifier = id;
    header.IdType = FDCAN_STANDARD_ID;
    header.TxFrameType = FDCAN_DATA_FRAME;
    header.DataLength = FDCAN_DLC_BYTES_8;
    header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    header.BitRateSwitch = fd_brs ? FDCAN_BRS_ON : FDCAN_BRS_OFF;
    header.FDFormat = fd_brs ? FDCAN_FD_CAN : FDCAN_CLASSIC_CAN;
    header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    return HAL_FDCAN_AddMessageToTxFifoQ(bus, &header, data.data()) == HAL_OK;
}
} // namespace motor_can
