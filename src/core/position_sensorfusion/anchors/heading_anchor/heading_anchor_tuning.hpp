#pragma once

#include <cstdint>

namespace heading_anchor_tuning
{
    constexpr std::uint32_t maximum_age_ms = 2000U;
    constexpr std::uint32_t estimated_delay_ms = 900U;
    constexpr std::int64_t minimum_distance_um = 300000;
    constexpr std::int64_t full_distance_um = 900000;
    constexpr std::uint8_t minimum_sample_count = 6U;
    constexpr std::uint8_t full_sample_count = 12U;
    constexpr std::uint8_t maximum_sample_count = 15U;
    constexpr std::uint32_t huber_delta_um = 20000U;
    constexpr std::uint32_t good_residual_um = 20000U;
    constexpr std::uint32_t zero_residual_um = 120000U;
    constexpr std::uint8_t iteration_count = 6U;
    constexpr std::uint16_t confidence_gain_permille = 2500U;
}
