#pragma once
#include <array>
#include <cstdint>

namespace can_bus {
inline constexpr std::array<std::uint8_t, 16> fdLengths{
    0, 1, 2, 3, 4, 5, 6, 7, 8, 12, 16, 20, 24, 32, 48, 64};
}
