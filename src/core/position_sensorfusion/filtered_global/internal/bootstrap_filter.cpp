#include "bootstrap_filter.hpp"

#include "../filtered_global_tuning.hpp"
#include "../../internal/confidence_math.hpp"
#include "../../internal/geometry_helpers.hpp"

namespace
{
    bool calculate_weighted_center(const filtered_global::sample *samples, std::uint8_t sample_count, filtered_global::sample &sample_out)
    {
        if (sample_count < filtered_global_tuning::bootstrap_min_sample_count)
        {
            return false;
        }

        std::int64_t weighted_x_um = 0;
        std::int64_t weighted_y_um = 0;
        std::int64_t weighted_z_um = 0;
        std::uint32_t confidence_sum = 0U;

        for (std::uint8_t index = 0U; index < sample_count; index++)
        {
            const std::uint32_t confidence = samples[index].confidence_position;
            weighted_x_um += samples[index].x_um * static_cast<std::int64_t>(confidence);
            weighted_y_um += samples[index].y_um * static_cast<std::int64_t>(confidence);
            weighted_z_um += samples[index].z_um * static_cast<std::int64_t>(confidence);
            confidence_sum += confidence;
        }

        if (confidence_sum == 0U)
        {
            return false;
        }

        sample_out = samples[0U];
        sample_out.x_um = weighted_x_um / static_cast<std::int64_t>(confidence_sum);
        sample_out.y_um = weighted_y_um / static_cast<std::int64_t>(confidence_sum);
        sample_out.z_um = weighted_z_um / static_cast<std::int64_t>(confidence_sum);
        sample_out.confidence_position = static_cast<std::uint16_t>(confidence_sum / static_cast<std::uint32_t>(sample_count));

        return true;
    }

    std::int64_t calculate_max_spread_um(const filtered_global::sample *samples, std::uint8_t sample_count, const filtered_global::sample &center_sample)
    {
        std::int64_t max_spread_um = 0;

        for (std::uint8_t index = 0U; index < sample_count; index++)
        {
            const std::int64_t delta_x_um = samples[index].x_um - center_sample.x_um;
            const std::int64_t delta_y_um = samples[index].y_um - center_sample.y_um;
            const std::int64_t spread_um = position_sensorfusion_internal::calculate_distance_um(delta_x_um, delta_y_um);

            if (spread_um > max_spread_um)
            {
                max_spread_um = spread_um;
            }
        }

        return max_spread_um;
    }
}

namespace bootstrap_filter
{
    bool build_initial_sample(const filtered_global::sample *samples, std::uint8_t sample_count, filtered_global::sample &sample_out, std::uint16_t &history_confidence_out)
    {
        filtered_global::sample center_sample = {};
        const bool has_center = calculate_weighted_center(samples, sample_count, center_sample);

        if (has_center == false)
        {
            history_confidence_out = 0U;
            return false;
        }

        const std::int64_t max_spread_um = calculate_max_spread_um(samples, sample_count, center_sample);
        history_confidence_out = position_sensorfusion_internal::range_to_confidence(max_spread_um, filtered_global_tuning::bootstrap_good_spread_um, filtered_global_tuning::bootstrap_zero_spread_um);
        center_sample.confidence_position = position_sensorfusion_internal::multiply_confidence(center_sample.confidence_position, history_confidence_out);

        if (center_sample.confidence_position < filtered_global_tuning::minimum_bootstrap_confidence)
        {
            return false;
        }

        sample_out = center_sample;
        return true;
    }
}
