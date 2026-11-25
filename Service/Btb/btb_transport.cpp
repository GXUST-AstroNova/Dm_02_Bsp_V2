#include "btb_transport.hpp"
#include "can_fd_length.hpp"
#include <algorithm>

namespace btb {
namespace {
constexpr unsigned chunk = 63;
std::uint8_t wireLength(std::size_t used) noexcept {
    const auto it = std::lower_bound(can_bus::fdLengths.begin(), can_bus::fdLengths.end(), used);
    return it == can_bus::fdLengths.end() ? 0 : *it;
}
}

std::uint8_t frameCount(std::size_t size) noexcept {
    if (!size || size > kMaxPayload) return 0;
    return size <= 64 ? 1 : 2;
}

std::uint8_t encodeFrame(unsigned index, std::span<const std::uint8_t> payload,
                         std::span<std::uint8_t, 64> frame) noexcept {
    const auto count = frameCount(payload.size());
    if (index >= count) return 0;
    const unsigned header = count == 2 ? 1 : 0;
    if (header) {
        frame[0] = static_cast<std::uint8_t>(0x10 + index);
        payload = index == 0 ? payload.first(chunk) : payload.subspan(chunk);
    }
    const auto length = wireLength(header + payload.size());
    const auto end = std::copy(payload.begin(), payload.end(), frame.begin() + header);
    std::fill(end, frame.begin() + length, 0);
    return length;
}

Reassembler::Result Reassembler::receive(std::span<const std::uint8_t> frame,
                                         std::span<std::uint8_t> payload, std::uint32_t now_ms) noexcept {
    const bool has_first = pending(now_ms);
    waiting_ = false;
    switch (frameCount(payload.size())) {
    case 1:
        if (frame.size() != wireLength(payload.size())) return Result::error;
        std::copy_n(frame.begin(), payload.size(), payload.begin());
        return Result::complete;
    case 2:
        if (frame.size() == 64 && frame.front() == 0x10) {
            std::copy_n(frame.begin() + 1, chunk, payload.begin());
            waiting_ = true;
            first_ms_ = now_ms;
            return Result::incomplete;
        }
        if (!has_first || frame.size() != wireLength(1 + payload.size() - chunk) ||
            frame.front() != 0x11) return Result::error;
        std::copy_n(frame.begin() + 1, payload.size() - chunk, payload.begin() + chunk);
        return Result::complete;
    default:
        return Result::error;
    }
}
}
