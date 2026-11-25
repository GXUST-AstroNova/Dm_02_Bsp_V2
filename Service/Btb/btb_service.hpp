#pragma once
#include "btb_message.hpp"
#include "btb_transport.hpp"
#include "hfdcan_bsp.h"

// 总线与消息由调用方持有，生命周期须覆盖服务。每个 ID 指定一个发送方。
class BtbService {
public:
    BtbService(FDCAN_HandleTypeDef& bus, BtbMessageBase& message) noexcept;
    ~BtbService();
    BtbService(const BtbService&) = delete;
    BtbService& operator=(const BtbService&) = delete;

    // 外设初始化后注册。重复调用成功，冲突、经典 CAN 或表满返回 false。
    bool init() noexcept;
    bool send() noexcept;
    bool online(std::uint32_t timeout_ms) const noexcept;

    struct Probe {
        const FDCAN_HandleTypeDef& bus;
        const BtbMessageBase& message;
        bool registered{};
        bool has_data{};
        bool awaiting_second_frame{};
        std::uint32_t last_rx_ms{};
        std::uint32_t rx_frames{}, rx_messages{}, rx_errors{};
        std::uint32_t tx_frames{}, tx_messages{}, tx_errors{};
    };
    [[nodiscard]] Probe probe() const noexcept;

private:
    void receive(std::span<const std::uint8_t> frame) noexcept;
    FDCAN_HandleTypeDef& bus_;
    BtbMessageBase& message_;
    btb::Reassembler rx_{};
    Probe state_;
};
