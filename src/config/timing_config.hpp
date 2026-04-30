#pragma once

#include <cstdint>

namespace timing_config
{
    constexpr std::uint32_t motion_mcu_communication_tick_interval_ms = 1U;
    constexpr std::uint32_t bluetooth_communication_tick_interval_ms = 10U;
    constexpr std::uint32_t global_positioning_tick_interval_ms = 20U;
    constexpr std::uint32_t position_sensorfusion_tick_interval_ms = 20U;
    constexpr std::uint32_t mission_tick_interval_ms = 20U;
    constexpr std::uint32_t pure_pursuit_tick_interval_ms = 20U;
}
