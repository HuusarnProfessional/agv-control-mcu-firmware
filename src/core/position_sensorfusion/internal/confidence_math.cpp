#include "confidence_math.hpp"

namespace position_sensorfusion_internal
{
    std::uint16_t smaller_confidence(std::uint16_t left, std::uint16_t right)
    {
        if (left < right)
        {
            return left;
        }

        return right;
    }

    std::uint16_t larger_confidence(std::uint16_t left, std::uint16_t right)
    {
        if (left > right)
        {
            return left;
        }

        return right;
    }

    std::uint16_t multiply_confidence(std::uint16_t left, std::uint16_t right)
    {
        const std::uint32_t multiplied = static_cast<std::uint32_t>(left) * static_cast<std::uint32_t>(right);
        return static_cast<std::uint16_t>(multiplied / full_confidence);
    }

    std::uint16_t apply_confidence_gain(std::uint16_t confidence, std::uint16_t gain_permille)
    {
        const std::uint32_t gained_confidence = static_cast<std::uint32_t>(confidence) * static_cast<std::uint32_t>(gain_permille) / full_confidence;

        if (gained_confidence > full_confidence)
        {
            return full_confidence;
        }

        return static_cast<std::uint16_t>(gained_confidence);
    }

    std::uint16_t range_to_confidence(std::int64_t value, std::int64_t full_value, std::int64_t zero_value)
    {
        if (value <= full_value)
        {
            return full_confidence;
        }

        if (value >= zero_value)
        {
            return 0U;
        }

        const std::int64_t range = zero_value - full_value;
        const std::int64_t remaining = zero_value - value;
        const std::int64_t confidence = remaining * static_cast<std::int64_t>(full_confidence) / range;

        return static_cast<std::uint16_t>(confidence);
    }

    std::uint16_t growth_to_confidence(std::int64_t value, std::int64_t zero_value, std::int64_t full_value)
    {
        if (value <= zero_value)
        {
            return 0U;
        }

        if (value >= full_value)
        {
            return full_confidence;
        }

        const std::int64_t range = full_value - zero_value;
        const std::int64_t progress = value - zero_value;
        const std::int64_t confidence = progress * static_cast<std::int64_t>(full_confidence) / range;

        return static_cast<std::uint16_t>(confidence);
    }

    std::uint16_t sample_count_to_confidence(std::uint8_t sample_count, std::uint8_t minimum_sample_count, std::uint8_t full_sample_count)
    {
        if (sample_count < minimum_sample_count)
        {
            return 0U;
        }

        if (sample_count >= full_sample_count)
        {
            return full_confidence;
        }

        const std::uint8_t range = full_sample_count - minimum_sample_count;
        const std::uint8_t progress = sample_count - minimum_sample_count;
        const std::uint16_t confidence = static_cast<std::uint16_t>(progress) * full_confidence / static_cast<std::uint16_t>(range);

        return confidence;
    }

    std::uint16_t age_to_confidence(std::uint32_t age_ms, std::uint32_t full_age_ms, std::uint32_t zero_age_ms)
    {
        if (age_ms <= full_age_ms)
        {
            return full_confidence;
        }

        if (age_ms >= zero_age_ms)
        {
            return 0U;
        }

        const std::uint32_t range_ms = zero_age_ms - full_age_ms;
        const std::uint32_t remaining_ms = zero_age_ms - age_ms;
        const std::uint32_t confidence = remaining_ms * full_confidence / range_ms;

        return static_cast<std::uint16_t>(confidence);
    }
}
