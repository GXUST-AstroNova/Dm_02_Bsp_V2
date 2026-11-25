#pragma once

#include <array>
#include <bit>
#include <cmath>
#include <concepts>
#include <cstdint>
#include <type_traits>
#include <tuple>
#include <utility>
#include <boost/pfr/core.hpp>

static_assert(std::endian::native == std::endian::little,
              "BTB 线上格式按小端定义，当前平台不是小端");

// ===========================================================================
// 语义字段类型
// ===========================================================================

/**
 * 速度字段：业务值是 float（单位 m/s），线上是 int16 小端、0.001 m/s/LSB。
 * 范围为[-32.768, 32.767] m/s。
 */
struct BtbMilliMps {
    float value = 0.0f;
};

/**
 * 范围枚举字段：业务值是 EnumT，线上是 uint8。
 * 编码/解码都要求值落在 [Min, Max] 内。
 */
template <typename EnumT, std::uint8_t Min, std::uint8_t Max>
struct BtbEnumU8 {
    static_assert(std::is_enum_v<EnumT>, "BtbEnumU8 的模板参数必须是枚举类型");
    static_assert(sizeof(EnumT) == 1U, "BtbEnumU8 的枚举底层类型必须为 1 字节");
    static_assert(Min <= Max, "BtbEnumU8 的 Min 必须 <= Max");
    EnumT value{};
};

/**
 * 保留字段：线上占 N 个字节，编码写 0，解码要求全 0。
 * 成员 dummy 只是为了让聚合体保持可反射，业务代码不应读取。
 */
template <std::uint8_t N>
struct BtbPad {
    static_assert(N >= 1U, "BtbPad 的字节数必须 >= 1");
    std::uint8_t dummy = 0U;
};

// ===========================================================================
// codec 主模板
// ===========================================================================

/**
 * 类型 T 的编码策略。
 * 普通聚合体递归逐字段编码，自定义字段可以特化 BtbCodec。
 */
template <typename T>
struct BtbCodec {
    static_assert(std::is_aggregate_v<T>, "字段需要聚合类型或 BtbCodec 特化");
    using Fields = decltype(boost::pfr::structure_tie(std::declval<T&>()));
    static constexpr std::size_t kSize = []<std::size_t... I>(std::index_sequence<I...>) {
        return (std::size_t{0} + ... + BtbCodec<std::remove_cvref_t<std::tuple_element_t<I, Fields>>>::kSize);
    }(std::make_index_sequence<std::tuple_size_v<Fields>>{});

    static bool encode(const T& value, std::uint8_t* out) noexcept {
        bool ok = true;
        boost::pfr::for_each_field(value, [&](const auto& field) {
            using Codec = BtbCodec<std::remove_cvref_t<decltype(field)>>;
            ok = ok && Codec::encode(field, out);
            out += Codec::kSize;
        });
        return ok;
    }

    static bool decode(const std::uint8_t* in, T& value) noexcept {
        bool ok = true;
        boost::pfr::for_each_field(value, [&](auto& field) {
            using Codec = BtbCodec<std::remove_cvref_t<decltype(field)>>;
            ok = ok && Codec::decode(in, field);
            in += Codec::kSize;
        });
        return ok;
    }
};

// ===========================================================================
// 内置 codec
// ===========================================================================

/** 算术类型：小端原样（bit_cast 到字节数组；平台小端由文件头 static_assert 保证） */
template <typename T>
    requires(std::is_arithmetic_v<T> && !std::is_same_v<T, bool>)
struct BtbCodec<T> {
    using value_type = T;
    static constexpr std::uint8_t kSize = static_cast<std::uint8_t>(sizeof(T));

    static bool encode(const T &v, std::uint8_t *out) noexcept {
        const auto bytes = std::bit_cast<std::array<std::uint8_t, sizeof(T)>>(v);
        for (std::uint8_t i = 0; i < kSize; ++i) {
            out[i] = bytes[i];
        }
        return true;
    }

    static bool decode(const std::uint8_t *in, T &v) noexcept {
        std::array<std::uint8_t, sizeof(T)> bytes{};
        for (std::uint8_t i = 0; i < kSize; ++i) {
            bytes[i] = in[i];
        }
        v = std::bit_cast<T>(bytes);
        return true;
    }
};

// 布尔字段固定占一个字节，解码只接受 0 和 1。
template <>
struct BtbCodec<bool> {
    static constexpr std::size_t kSize = 1;
    static bool encode(bool value, std::uint8_t* out) noexcept {
        *out = value ? 1U : 0U;
        return true;
    }
    static bool decode(const std::uint8_t* in, bool& value) noexcept {
        if (*in > 1U) return false;
        value = *in != 0U;
        return true;
    }
};

/** 0.001 m/s/LSB 的 int16 速度；业务值 float，越界/NaN/Inf 拒绝编码 */
template <>
struct BtbCodec<BtbMilliMps> {
    using value_type = BtbMilliMps;
    static constexpr std::uint8_t kSize = 2U;
    static constexpr float kMin = -32.768f;
    static constexpr float kMax = 32.767f;
    static constexpr float kScale = 1000.0f;

    static bool encode(const value_type &v, std::uint8_t *out) noexcept {
        if (!std::isfinite(v.value) || v.value < kMin || v.value > kMax) {
            return false;
        }
        const auto raw = static_cast<std::int32_t>(std::lroundf(v.value * kScale));
        out[0] = static_cast<std::uint8_t>(raw & 0xFF);
        out[1] = static_cast<std::uint8_t>((raw >> 8) & 0xFF);
        return true;
    }

    static bool decode(const std::uint8_t *in, value_type &v) noexcept {
        const auto raw = static_cast<std::uint16_t>(
            static_cast<std::uint16_t>(in[0]) |
            static_cast<std::uint16_t>(static_cast<std::uint16_t>(in[1]) << 8));
        // 用 bit_cast 得到二补码，避免 unsigned -> signed 的实现定义行为。
        v.value = static_cast<float>(std::bit_cast<std::int16_t>(raw)) / kScale;
        return true;
    }
};

/** 范围枚举：线上 1 字节，要求 Min..Max */
template <typename EnumT, std::uint8_t Min, std::uint8_t Max>
struct BtbCodec<BtbEnumU8<EnumT, Min, Max>> {
    using value_type = BtbEnumU8<EnumT, Min, Max>;
    static constexpr std::uint8_t kSize = 1U;

    static bool encode(const value_type &v, std::uint8_t *out) noexcept {
        const auto raw = static_cast<std::uint8_t>(v.value);
        if (raw < Min || raw > Max) {
            return false;
        }
        out[0] = raw;
        return true;
    }

    static bool decode(const std::uint8_t *in, value_type &v) noexcept {
        if (in[0] < Min || in[0] > Max) {
            return false;
        }
        v.value = static_cast<EnumT>(in[0]);
        return true;
    }
};

/** 保留字节：编码写 0，解码校验全 0 */
template <std::uint8_t N>
struct BtbCodec<BtbPad<N>> {
    using value_type = BtbPad<N>;
    static constexpr std::uint8_t kSize = N;

    static bool encode(const value_type &, std::uint8_t *out) noexcept {
        for (std::uint8_t i = 0; i < kSize; ++i) {
            out[i] = 0U;
        }
        return true;
    }

    static bool decode(const std::uint8_t *in, value_type &v) noexcept {
        for (std::uint8_t i = 0; i < kSize; ++i) {
            if (in[i] != 0U) {
                return false;
            }
        }
        v.dummy = 0U;
        return true;
    }
};

template<class T, std::size_t N>
struct BtbCodec<std::array<T, N>> {
    using Element = BtbCodec<T>;
    static constexpr std::size_t kSize = N * Element::kSize;

    static bool encode(const std::array<T, N>& values, std::uint8_t* out) noexcept {
        for (const auto& value : values) {
            if (!Element::encode(value, out)) return false;
            out += Element::kSize;
        }
        return true;
    }
    static bool decode(const std::uint8_t* in, std::array<T, N>& values) noexcept {
        for (auto& value : values) {
            if (!Element::decode(in, value)) return false;
            in += Element::kSize;
        }
        return true;
    }
};
