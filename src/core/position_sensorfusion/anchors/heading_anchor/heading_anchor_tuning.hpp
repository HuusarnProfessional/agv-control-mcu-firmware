#pragma once

#include <cstdint>

namespace heading_anchor_tuning
{
    constexpr std::uint32_t maximum_age_ms = 2000U;
    constexpr std::uint32_t estimated_delay_ms = 500U;
    // Position anchors can switch branch roughly once per second, so heading
    // needs to converge within a much shorter same-branch travel segment.
    constexpr std::int64_t minimum_distance_um = 100000;
    constexpr std::int64_t full_distance_um = 300000;
    constexpr std::uint8_t minimum_sample_count = 4U;
    constexpr std::uint8_t full_sample_count = 8U;
    constexpr std::uint8_t maximum_sample_count = 15U;
    constexpr std::uint32_t huber_delta_um = 20000U;
    constexpr std::uint32_t good_residual_um = 20000U;
    constexpr std::uint32_t zero_residual_um = 120000U;
    constexpr std::uint8_t iteration_count = 6U;
    constexpr std::uint16_t confidence_gain_permille = 1000U;
    constexpr std::uint32_t minimum_trajectory_distance_um = 100000U;
    constexpr std::uint32_t maximum_local_heading_span_urad = 350000U;
    constexpr std::uint32_t maximum_local_line_residual_um = 90000U;
}
