//
// 通用 USART 故障分发 / 收养入口
//
// 本文件只做 BSP 机制：HAL 回调分发、端口表、错误 pending、外设级恢复原语、
// 调试口预留。业务层决定恢复策略和调用周期。
//
#ifndef DM_02_BSP_UART_BSP_H
#define DM_02_BSP_UART_BSP_H

#include <cstdint>

#include "usart.h"

namespace uart_bsp {

constexpr std::uint8_t kMaxPorts = 8U;
constexpr std::uint8_t kMaxRaw = 8U;
constexpr std::uint8_t kMaxReserved = 4U;

/** 已注册的 UART 接收端口。回调均可为空。 */
struct Port {
    UART_HandleTypeDef *handle = nullptr;
    void *ctx = nullptr;
    void (*rx_event)(void *ctx, UART_HandleTypeDef *huart, std::uint16_t size) = nullptr;
    void (*on_error)(void *ctx, UART_HandleTypeDef *huart) = nullptr;
    bool (*is_stale)(void *ctx, std::uint32_t now_ms) = nullptr;
    bool (*recover)(void *ctx, bool hard) = nullptr;
    std::uint8_t id = 0xFFU;
};

int8_t Register(Port port) noexcept;
void Unregister(void *ctx) noexcept;

/** 预留调试口：错误回调只计数，不做 raw 收养/恢复。 */
void Reserve(UART_HandleTypeDef *huart) noexcept;

const Port *Get(std::uint8_t id) noexcept;
std::uint8_t Count() noexcept;
bool Port_Error_Pending(std::uint8_t id) noexcept;
void Port_Clear_Pending(std::uint8_t id) noexcept;
std::uint32_t Port_Error_Count(std::uint8_t id) noexcept;

std::uint8_t Raw_Count() noexcept;
UART_HandleTypeDef *Raw_Handle(std::uint8_t index) noexcept;
bool Raw_Error_Pending(std::uint8_t index) noexcept;
void Raw_Clear_Pending(std::uint8_t index) noexcept;
std::uint32_t Raw_Error_Count(std::uint8_t index) noexcept;

/** 外设级恢复（AbortReceive + DeInit + Init）；不重挂数据通路。 */
bool Restore_Raw(UART_HandleTypeDef *huart) noexcept;

std::uint32_t Total_Error_Count() noexcept;
std::uint32_t Unhandled_Error_Count() noexcept;

}  // namespace uart_bsp

#endif  // DM_02_BSP_UART_BSP_H
