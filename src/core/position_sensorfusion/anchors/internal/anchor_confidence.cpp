#include "anchor_confidence.hpp"

#include "../../internal/robust_statistics.hpp"

namespace anchor_confidence
{
    std::uint16_t median_position_confidence(const filtered_global::sample *samples, std::uint8_t sample_count)
    {
        std::int64_t confidence_values[64] = {};
        std::uint8_t confidence_count = 0U;

        for (std::uint8_t index = 0U; index < sample_count; index++)
        {
            if (samples[index].valid == false)
            {
                continue;
            }

            confidence_values[confidence_count] = samples[index].confidence_position;
            confidence_count++;
        }

        if (confidence_count == 0U)
        {
            return 0U;
        }

        return static_cast<std::uint16_t>(position_sensorfusion_internal::median_value(confidence_values, confidence_count));
    }

    std::uint32_t median_residual_um(const std::uint32_t *residuals_um, std::uint8_t sample_count)
    {
        std::int64_t residual_values[64] = {};

        for (std::uint8_t index = 0U; index < sample_count; index++)
        {
            residual_values[index] = residuals_um[index];
        }

        return static_cast<std::uint32_t>(position_sensorfusion_internal::median_value(residual_values, sample_count));
    }
}
