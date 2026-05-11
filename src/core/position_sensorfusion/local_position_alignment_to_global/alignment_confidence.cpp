#include "alignment_confidence.hpp"

#include <cstdint>

namespace
{
    constexpr std::uint16_t full_confidence = 1000U;

    constexpr std::uint32_t age_full_score_ms = 150U;
    constexpr std::uint32_t age_zero_score_ms = 1000U;

    std::uint32_t get_age_ms(std::uint32_t now_ms, std::uint32_t start_time_ms)
    {
        if (now_ms < start_time_ms)
        {
            return 0U;
        }

        return now_ms - start_time_ms;
    }

    std::uint16_t age_to_score_factor(std::uint32_t age_ms)
    {
        if (age_ms <= age_full_score_ms)
        {
            return full_confidence;
        }

        if (age_ms >= age_zero_score_ms)
        {
            return 0U;
        }

        const std::uint32_t age_range_ms = age_zero_score_ms - age_full_score_ms;
        const std::uint32_t age_over_full_ms = age_ms - age_full_score_ms;
        const std::uint32_t remaining_ms = age_range_ms - age_over_full_ms;
        const std::uint32_t score = remaining_ms * full_confidence / age_range_ms;

        return static_cast<std::uint16_t>(score);
    }
}

namespace alignment_confidence
{
    std::uint16_t multiply(std::uint16_t left, std::uint16_t right)
    {
        const std::uint32_t multiplied = static_cast<std::uint32_t>(left) * static_cast<std::uint32_t>(right);
        const std::uint32_t scaled = multiplied / full_confidence;

        return static_cast<std::uint16_t>(scaled);
    }

    std::uint16_t calculate_anchor_score(const global_position_heading::output_snapshot &global_position, std::uint32_t now_ms)
    {
        if (global_position.has_position == false)
        {
            return 0U;
        }

        const std::uint32_t age_ms = get_age_ms(now_ms, global_position.received_time_ms);
        const std::uint16_t age_score = age_to_score_factor(age_ms);
        const std::uint16_t anchor_score = multiply(global_position.confidence_position, age_score);

        return anchor_score;
    }
}