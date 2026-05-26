#include "adaptive_low_pass.hpp"

#include "../filtered_global_tuning.hpp"
#include "../../internal/confidence_math.hpp"
#include "../../internal/geometry_helpers.hpp"

namespace
{
    std::int64_t calculate_weighted_step_um(std::int64_t difference_um, std::uint16_t current_confidence, std::uint16_t sample_confidence)
    {
        const std::uint32_t total_confidence = static_cast<std::uint32_t>(current_confidence) + static_cast<std::uint32_t>(sample_confidence);

        if (total_confidence == 0U)
        {
            return 0;
        }

        return difference_um * static_cast<std::int64_t>(sample_confidence) / static_cast<std::int64_t>(total_confidence);
    }

    std::int64_t calculate_max_step_um(const filtered_global::sample &previous_sample, const filtered_global::sample &raw_sample)
    {
        const std::uint32_t time_delta_ms = position_sensorfusion_internal::elapsed_ms(raw_sample.received_time_ms, previous_sample.received_time_ms);
        return filtered_global_tuning::position_step_base_um + (filtered_global_tuning::position_step_speed_um_per_ms * static_cast<std::int64_t>(time_delta_ms));
    }

    void limit_step(std::int64_t &step_x_um, std::int64_t &step_y_um, std::int64_t maximum_step_um)
    {
        if (maximum_step_um <= 0)
        {
            step_x_um = 0;
            step_y_um = 0;
            return;
        }

        const std::int64_t step_distance_um = position_sensorfusion_internal::calculate_distance_um(step_x_um, step_y_um);

        if (step_distance_um <= maximum_step_um)
        {
            return;
        }

        if (step_distance_um == 0)
        {
            return;
        }

        step_x_um = step_x_um * maximum_step_um / step_distance_um;
        step_y_um = step_y_um * maximum_step_um / step_distance_um;
    }
}

namespace adaptive_low_pass
{
    filtered_global::sample update_position(const filtered_global::sample &previous_sample, const filtered_global::sample &raw_sample, std::uint16_t filtered_confidence)
    {
        filtered_global::sample filtered_sample = raw_sample;
        std::uint16_t memory_confidence = previous_sample.confidence_position;

        if (memory_confidence > filtered_global_tuning::low_pass_memory_confidence_cap)
        {
            memory_confidence = filtered_global_tuning::low_pass_memory_confidence_cap;
        }

        std::int64_t step_x_um = calculate_weighted_step_um(raw_sample.x_um - previous_sample.x_um, memory_confidence, filtered_confidence);
        std::int64_t step_y_um = calculate_weighted_step_um(raw_sample.y_um - previous_sample.y_um, memory_confidence, filtered_confidence);
        limit_step(step_x_um, step_y_um, calculate_max_step_um(previous_sample, raw_sample));

        filtered_sample.x_um = previous_sample.x_um + step_x_um;
        filtered_sample.y_um = previous_sample.y_um + step_y_um;
        filtered_sample.z_um = previous_sample.z_um + calculate_weighted_step_um(raw_sample.z_um - previous_sample.z_um, memory_confidence, filtered_confidence);
        filtered_sample.confidence_position = filtered_confidence;

        return filtered_sample;
    }
}
