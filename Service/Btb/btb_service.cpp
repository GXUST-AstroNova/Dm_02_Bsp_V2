#include "btb_service.hpp"
#include <array>

BtbService::BtbService(FDCAN_HandleTypeDef& bus, BtbMessageBase& message) noexcept
    : bus_(bus), message_(message), state_{bus, message} {}

BtbService::~BtbService() {
    if (state_.registered) can_bus::unsubscribeFd(this);
}

bool BtbService::init() noexcept {
    if (state_.registered) return true;
    state_.registered = can_bus::subscribeFd(bus_, message_.canId(), this,
        [](void* context, std::span<const std::uint8_t> frame) noexcept {
            static_cast<BtbService*>(context)->receive(frame);
        });
    return state_.registered;
}

bool BtbService::send() noexcept {
    std::array<std::uint8_t, btb::kMaxPayload> storage;
    if (!state_.registered || message_.payloadSize() > storage.size()) {
        ++state_.tx_errors;
        return false;
    }
    const auto payload = std::span(storage).first(message_.payloadSize());
    if (!message_.encode(payload)) {
        ++state_.tx_errors;
        return false;
    }
    const auto count = btb::frameCount(payload.size());
    if (!count || count > can_bus::txSlots(bus_)) {
        ++state_.tx_errors;
        return false;
    }
    std::array<std::uint8_t, 64> frame{};
    for (unsigned i = 0; i < count; ++i) {
        const auto length = btb::encodeFrame(i, payload, frame);
        if (!can_bus::enqueueFd(bus_, message_.canId(), frame, length)) {
            ++state_.tx_errors;
            return false;
        }
        ++state_.tx_frames;
    }
    ++state_.tx_messages;
    return true;
}

void BtbService::receive(std::span<const std::uint8_t> frame) noexcept {
    ++state_.rx_frames;
    const auto now = HAL_GetTick();
    const auto buffer = message_.buffer();
    const auto result = rx_.receive(frame, buffer, now);
    if (result == btb::Reassembler::Result::incomplete) return;
    if (result == btb::Reassembler::Result::error || !message_.decode(buffer)) {
        ++state_.rx_errors;
        return;
    }
    state_.last_rx_ms = now;
    state_.has_data = true;
    ++state_.rx_messages;
}

BtbService::Probe BtbService::probe() const noexcept {
    IrqGuard guard;
    Probe snapshot = state_;
    snapshot.awaiting_second_frame = rx_.pending(HAL_GetTick());
    return snapshot;
}

bool BtbService::online(std::uint32_t timeout_ms) const noexcept {
    const auto snapshot = probe();
    return snapshot.has_data && HAL_GetTick() - snapshot.last_rx_ms <= timeout_ms;
}
