#pragma once
#include "btb_codec.hpp"
#include "btb_defs.hpp"
#include "irq_guard.hpp"
#include <span>

// 一个消息对象拥有类型化载荷与对应长度的接收缓冲。
class BtbMessageBase {
public:
    virtual ~BtbMessageBase() = default;
    BtbMessageBase(const BtbMessageBase&) = delete;
    BtbMessageBase& operator=(const BtbMessageBase&) = delete;
    std::uint32_t canId() const noexcept { return id_; }
    std::size_t payloadSize() const noexcept { return rx_.size(); }
    std::span<std::uint8_t> buffer() noexcept { return rx_; }
    virtual bool encode(std::span<std::uint8_t> out) const noexcept = 0;
    virtual bool decode(std::span<const std::uint8_t> in) noexcept = 0;
protected:
    BtbMessageBase(std::uint32_t id, std::span<std::uint8_t> rx) noexcept : id_(id), rx_(rx) {}
private:
    std::uint32_t id_;
    std::span<std::uint8_t> rx_;
};

template<class Payload>
class BtbMessage final : public BtbMessageBase {
    using Codec = BtbCodec<Payload>;
public:
    static constexpr std::size_t kPayloadSize = Codec::kSize;
    static_assert(kPayloadSize >= 1 && kPayloadSize <= btb::kMaxPayload);
    static_assert(std::is_trivially_copyable_v<Payload>, "中断快照要求载荷可平凡复制");

    BtbMessage(std::uint32_t id, Payload initial) noexcept
        : BtbMessageBase(id, std::span<std::uint8_t>{rx_storage_, kPayloadSize}), payload_(initial) {}

    void set(const Payload& payload) noexcept {
        IrqGuard guard;
        payload_ = payload;
    }
    [[nodiscard]] Payload value() const noexcept {
        IrqGuard guard;
        return payload_;
    }
    bool encode(std::span<std::uint8_t> out) const noexcept override {
        return out.size() == kPayloadSize && Codec::encode(value(), out.data());
    }
    bool decode(std::span<const std::uint8_t> in) noexcept override {
        Payload next{};
        if (in.size() != kPayloadSize || !Codec::decode(in.data(), next)) return false;
        set(next);
        return true;
    }
private:
    std::uint8_t rx_storage_[kPayloadSize]{};
    Payload payload_;
};
