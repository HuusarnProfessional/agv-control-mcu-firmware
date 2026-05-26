#pragma once

#include <cstdint>

namespace position_sensorfusion_internal
{
    constexpr std::uint16_t full_confidence = 1000U;

    std::uint16_t smaller_confidence(std::uint16_t left, std::uint16_t right);

    std::uint16_t larger_confidence(std::uint16_t left, std::uint16_t right);

    std::uint16_t multiply_confidence(std::uint16_t left, std::uint16_t right);

    std::uint16_t geometric_mean_confidence(std::uint16_t first, std::uint16_t second, std::uint16_t third);

    std::uint16_t apply_confidence_gain(std::uint16_t confidence, std::uint16_t gain_permille);

    std::uint16_t range_to_confidence(std::int64_t value, std::int64_t full_value, std::int64_t zero_value);

    std::uint16_t growth_to_confidence(std::int64_t value, std::int64_t zero_value, std::int64_t full_value);

    std::uint16_t sample_count_to_confidence(std::uint8_t sample_count, std::uint8_t minimum_sample_count, std::uint8_t full_sample_count);

    std::uint16_t age_to_confidence(std::uint32_t age_ms, std::uint32_t full_age_ms, std::uint32_t zero_age_ms);
}
