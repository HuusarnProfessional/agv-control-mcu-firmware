#pragma once

#include <cstdint>

namespace position_anchor_tuning
{
    constexpr std::uint32_t maximum_age_ms = 1100U;
    constexpr std::uint32_t estimated_delay_ms = 250U;
    constexpr std::uint8_t minimum_sample_count = 3U;
    constexpr std::uint8_t full_sample_count = 7U;
    constexpr std::uint8_t maximum_sample_count = 7U;
    constexpr std::uint32_t huber_delta_um = 80000U;
    constexpr std::uint32_t good_residual_um = 50000U;
    constexpr std::uint32_t zero_residual_um = 300000U;
    constexpr std::uint8_t iteration_count = 5U;
    constexpr std::uint16_t confidence_gain_permille = 2500U;
    constexpr std::uint32_t minimum_trajectory_distance_um = 100000U;
    constexpr std::uint32_t maximum_local_heading_span_urad = 500000U;
    constexpr std::uint32_t maximum_local_line_residual_um = 120000U;
    constexpr std::uint8_t long_heading_gate_sample_count = 10U;
    constexpr std::uint32_t long_heading_gate_maximum_span_urad = 175000U;
}
