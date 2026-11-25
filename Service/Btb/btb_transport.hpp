#pragma once
#include "btb_defs.hpp"
#include <span>

namespace btb {
// 单帧 1..64 字节。65..112 字节分两帧，每帧的首字节为 0x10 或 0x11。
std::uint8_t frameCount(std::size_t size) noexcept;
std::uint8_t encodeFrame(unsigned index, std::span<const std::uint8_t> payload,
                         std::span<std::uint8_t, 64> frame) noexcept;

class Reassembler {
public:
    enum class Result { incomplete, complete, error };
    Result receive(std::span<const std::uint8_t> frame, std::span<std::uint8_t> payload,
                   std::uint32_t now_ms) noexcept;
    bool pending(std::uint32_t now_ms) const noexcept {
        return waiting_ && now_ms - first_ms_ <= kReassemblyTimeoutMs;
    }
private:
    bool waiting_{};
    std::uint32_t first_ms_{};
};
}
