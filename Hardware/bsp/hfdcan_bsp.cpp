#include "hfdcan_bsp.h"
#include "dm_motor.hpp"
#include "m3508.hpp"
#include <algorithm>

namespace {
uint32_t rxNotification(FDCAN_HandleTypeDef* bus) noexcept {
    return bus->Init.RxFifo0ElmtsNbr ? FDCAN_IT_RX_FIFO0_NEW_MESSAGE : FDCAN_IT_RX_FIFO1_NEW_MESSAGE;
}

bool startWithInterrupt(FDCAN_HandleTypeDef* bus) noexcept {
    if (HAL_FDCAN_Start(bus) != HAL_OK) return false;
    if (HAL_FDCAN_ActivateNotification(bus, rxNotification(bus) | FDCAN_IT_BUS_OFF, 0) == HAL_OK) {
        return true;
    }
    HAL_FDCAN_Stop(bus);
    return false;
}

void drain(FDCAN_HandleTypeDef* bus, uint32_t fifo) noexcept {
    // FDCAN2 FIFO1 holds up to 64 bytes. HAL copies before we can inspect the DLC.
    std::array<uint8_t, 64> buffer{};
    FDCAN_RxHeaderTypeDef header{};
    while (HAL_FDCAN_GetRxFifoFillLevel(bus, fifo)) {
        if (HAL_FDCAN_GetRxMessage(bus, fifo, &header, buffer.data()) != HAL_OK) break;
        if (header.IdType != FDCAN_STANDARD_ID || header.RxFrameType != FDCAN_DATA_FRAME ||
            header.DataLength != FDCAN_DLC_BYTES_8) continue;
        const bool classic = header.FDFormat == FDCAN_CLASSIC_CAN && header.BitRateSwitch == FDCAN_BRS_OFF;
        const bool fd_brs = header.FDFormat == FDCAN_FD_CAN && header.BitRateSwitch == FDCAN_BRS_ON;
        motor_can::Payload payload{};
        std::copy_n(buffer.begin(), payload.size(), payload.begin());
        if (classic && DjiMotorGroup::dispatch(bus, header.Identifier, payload)) continue;
        if (fd_brs || classic) {
            DmMotor::dispatch(bus, header.Identifier, payload, fd_brs);
        }
    }
}
} // namespace

bool can_bus::start(FDCAN_HandleTypeDef* bus) noexcept {
    if (!bus || (!bus->Init.RxFifo0ElmtsNbr && !bus->Init.RxFifo1ElmtsNbr)) return false;
    if (motor_can::usesFdBrs(bus)) {
        // Secondary sample point offset in FDCAN kernel clock periods (12 at 5 Mbit/s).
        const uint32_t offset = bus->Init.DataPrescaler * (1 + bus->Init.DataTimeSeg1);
        if (HAL_FDCAN_ConfigTxDelayCompensation(bus, offset, 0) != HAL_OK ||
            HAL_FDCAN_EnableTxDelayCompensation(bus) != HAL_OK) return false;
    }
    FDCAN_FilterTypeDef filter{};
    filter.IdType = FDCAN_STANDARD_ID;
    filter.FilterType = FDCAN_FILTER_MASK;
    filter.FilterConfig = bus->Init.RxFifo0ElmtsNbr ? FDCAN_FILTER_TO_RXFIFO0 : FDCAN_FILTER_TO_RXFIFO1;
    return HAL_FDCAN_ConfigFilter(bus, &filter) == HAL_OK &&
           HAL_FDCAN_ConfigGlobalFilter(bus, FDCAN_REJECT, FDCAN_REJECT,
                                        FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE) == HAL_OK &&
           startWithInterrupt(bus);
}

bool can_bus::Soft_Restart(FDCAN_HandleTypeDef* bus) noexcept {
    if (!bus) return false;
    if (bus->State == HAL_FDCAN_STATE_BUSY) {
        HAL_FDCAN_AbortTxRequest(bus, 0xffffffffu);
        if (HAL_FDCAN_Stop(bus) != HAL_OK) return false;
    }
    return startWithInterrupt(bus);
}

bool can_bus::Hard_Restart(FDCAN_HandleTypeDef* bus) noexcept {
    return bus && HAL_FDCAN_DeInit(bus) == HAL_OK && HAL_FDCAN_Init(bus) == HAL_OK && start(bus);
}

extern "C" void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef* bus, uint32_t flags) {
    if (flags & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) drain(bus, FDCAN_RX_FIFO0);
}

extern "C" void HAL_FDCAN_RxFifo1Callback(FDCAN_HandleTypeDef* bus, uint32_t flags) {
    if (flags & FDCAN_IT_RX_FIFO1_NEW_MESSAGE) drain(bus, FDCAN_RX_FIFO1);
}

// Bus_Off is recovered in place. Clearing CCCR.INIT (HAL_FDCAN_Start) puts the
// controller back into the bus-off recovery sequence: it waits for 129
// occurrences of 11 recessive bits, then resumes transmission on its own.
// Soft_Restart aborts the stale Tx requests first, so no queued motor command is
// replayed onto the bus after the fault. If the restart itself fails (the HAL
// state machine drops to ERROR after a Stop timeout), the bus stays silent and
// the application has to escalate to can_bus::Hard_Restart.
extern "C" void HAL_FDCAN_ErrorStatusCallback(FDCAN_HandleTypeDef* bus, uint32_t flags) {
    if (flags & FDCAN_IT_BUS_OFF) {
        (void)can_bus::Soft_Restart(bus);
    }
}
