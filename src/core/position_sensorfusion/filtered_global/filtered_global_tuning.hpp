#pragma once

#include <cstdint>

namespace filtered_global_tuning
{
    constexpr std::uint8_t bootstrap_max_sample_count = 6U;
    constexpr std::uint8_t bootstrap_min_sample_count = 3U;
    constexpr std::uint16_t minimum_bootstrap_confidence = 150U;
    constexpr std::int64_t bootstrap_good_spread_um = 250000;
    constexpr std::int64_t bootstrap_zero_spread_um = 1000000;

    constexpr std::uint8_t history_size = 64U;
    constexpr std::uint16_t minimum_accepted_confidence = 120U;
    constexpr std::uint16_t minimum_tracking_confidence = 80U;
    constexpr std::int64_t maximum_robot_speed_um_per_ms = 1500;
    constexpr std::int64_t physical_jump_margin_um = 500000;
    constexpr std::int64_t prediction_good_residual_um = 150000;
    constexpr std::int64_t prediction_zero_residual_um = 1200000;
    constexpr std::int64_t speed_mad_margin_um_per_ms = 700;
    constexpr std::uint8_t minimum_speed_count_for_mad = 3U;

    constexpr std::int64_t position_step_base_um = 50000;
    constexpr std::int64_t position_step_speed_um_per_ms = 500;
    constexpr std::uint16_t low_pass_memory_confidence_cap = 400U;
    constexpr std::uint32_t position_confidence_full_age_ms = 250U;
    constexpr std::uint32_t position_confidence_zero_age_ms = 2500U;

    constexpr std::uint8_t local_history_size = 128U;
    constexpr std::uint32_t local_history_max_match_error_ms = 150U;
}
