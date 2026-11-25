#pragma once

#include <concepts>
#include <cstdint>
#include <cstdlib>

class DmCanId {
public:
    // Validate the original integer before narrowing. A non-constant argument
    // or a call to abort() in immediate evaluation is a compilation error.
    template<std::integral Integer> requires (!std::same_as<Integer, bool>)
    consteval DmCanId(Integer id) : value_(static_cast<uint16_t>(id)) {
        if (id < 0 || id > 0x7ff) std::abort(); // DM CAN ID must be in [0, 0x7ff].
    }

    constexpr uint16_t value() const noexcept { return value_; }
private:
    uint16_t value_;
};

