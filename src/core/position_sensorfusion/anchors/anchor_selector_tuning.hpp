#pragma once

#include <cstdint>

namespace anchor_selector_tuning
{
    constexpr std::uint16_t minimum_anchor_confidence = 300U;
    constexpr std::uint16_t reference_switch_margin_percent = 10U;
    constexpr std::uint16_t pending_switch_margin_percent = 25U;
    constexpr std::uint32_t minimum_request_interval_ms = 1000U;
    constexpr std::uint32_t pending_request_timeout_ms = 3000U;
    constexpr std::uint32_t settling_time_ms = 500U;
    constexpr std::uint16_t maximum_reference_pose_age_steps = 2048U;
    constexpr std::int64_t safe_min_x_um = 400000;
    constexpr std::int64_t safe_max_x_um = 5000000;
    constexpr std::int64_t safe_min_y_um = 400000;
    constexpr std::int64_t safe_max_y_um = 3200000;
    constexpr std::int64_t maximum_anchor_position_jump_um = 350000;
    constexpr std::int32_t maximum_heading_delta_urad = 523599;
}
