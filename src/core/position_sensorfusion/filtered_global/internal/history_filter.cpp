#include "history_filter.hpp"

#include "../filtered_global_tuning.hpp"
#include "../../internal/confidence_math.hpp"
#include "../../internal/geometry_helpers.hpp"
#include "../../internal/robust_statistics.hpp"

namespace
{
    std::uint32_t calculate_time_delta_ms(const filtered_global::sample &older_sample, const filtered_global::sample &newer_sample)
    {
        return position_sensorfusion_internal::elapsed_ms(newer_sample.received_time_ms, older_sample.received_time_ms);
    }

    std::int64_t calculate_sample_distance_um(const filtered_global::sample &older_sample, const filtered_global::sample &newer_sample)
    {
        return position_sensorfusion_internal::calculate_distance_um(newer_sample.x_um - older_sample.x_um, newer_sample.y_um - older_sample.y_um);
    }

    std::int64_t calculate_speed_um_per_ms(const filtered_global::sample &older_sample, const filtered_global::sample &newer_sample)
    {
        const std::uint32_t time_delta_ms = calculate_time_delta_ms(older_sample, newer_sample);

        if (time_delta_ms == 0U)
        {
            return 0;
        }

        return calculate_sample_distance_um(older_sample, newer_sample) / static_cast<std::int64_t>(time_delta_ms);
    }

    std::uint16_t calculate_physical_confidence(const filtered_global_history::state &history, const filtered_global::sample &raw_sample)
    {
        filtered_global::sample newest_sample = {};

        if (filtered_global_history::newest(history, newest_sample) == false)
        {
            return position_sensorfusion_internal::full_confidence;
        }

        const std::uint32_t time_delta_ms = calculate_time_delta_ms(newest_sample, raw_sample);

        if (time_delta_ms == 0U)
        {
            return 0U;
        }

        const std::int64_t distance_um = calculate_sample_distance_um(newest_sample, raw_sample);
        const std::int64_t allowed_distance_um = (filtered_global_tuning::maximum_robot_speed_um_per_ms * static_cast<std::int64_t>(time_delta_ms)) + filtered_global_tuning::physical_jump_margin_um;
        const std::int64_t zero_confidence_distance_um = allowed_distance_um + filtered_global_tuning::physical_jump_margin_um;

        return position_sensorfusion_internal::range_to_confidence(distance_um, allowed_distance_um, zero_confidence_distance_um);
    }

    std::uint16_t calculate_prediction_confidence(const filtered_global_history::state &history, const filtered_global::sample &raw_sample)
    {
        filtered_global::sample newest_sample = {};
        filtered_global::sample older_sample = {};

        if (filtered_global_history::read(history, 0U, newest_sample) == false)
        {
            return position_sensorfusion_internal::full_confidence;
        }

        if (filtered_global_history::read(history, 1U, older_sample) == false)
        {
            return position_sensorfusion_internal::full_confidence;
        }

        const std::uint32_t previous_time_delta_ms = calculate_time_delta_ms(older_sample, newest_sample);
        const std::uint32_t raw_time_delta_ms = calculate_time_delta_ms(newest_sample, raw_sample);

        if (previous_time_delta_ms == 0U)
        {
            return position_sensorfusion_internal::full_confidence;
        }

        if (raw_time_delta_ms == 0U)
        {
            return 0U;
        }

        const double velocity_x_um_per_ms = static_cast<double>(newest_sample.x_um - older_sample.x_um) / static_cast<double>(previous_time_delta_ms);
        const double velocity_y_um_per_ms = static_cast<double>(newest_sample.y_um - older_sample.y_um) / static_cast<double>(previous_time_delta_ms);
        const double predicted_x_um = static_cast<double>(newest_sample.x_um) + (velocity_x_um_per_ms * static_cast<double>(raw_time_delta_ms));
        const double predicted_y_um = static_cast<double>(newest_sample.y_um) + (velocity_y_um_per_ms * static_cast<double>(raw_time_delta_ms));
        const std::int64_t residual_x_um = raw_sample.x_um - static_cast<std::int64_t>(predicted_x_um);
        const std::int64_t residual_y_um = raw_sample.y_um - static_cast<std::int64_t>(predicted_y_um);
        const std::int64_t residual_um = position_sensorfusion_internal::calculate_distance_um(residual_x_um, residual_y_um);

        return position_sensorfusion_internal::range_to_confidence(residual_um, filtered_global_tuning::prediction_good_residual_um, filtered_global_tuning::prediction_zero_residual_um);
    }

    std::uint8_t collect_history_speeds(const filtered_global_history::state &history, std::int64_t *speeds_out, std::uint8_t max_count)
    {
        std::uint8_t speed_count = 0U;

        for (std::uint8_t index = 0U; index + 1U < history.count; index++)
        {
            if (speed_count >= max_count)
            {
                return speed_count;
            }

            filtered_global::sample newer_sample = {};
            filtered_global::sample older_sample = {};

            if (filtered_global_history::read(history, index, newer_sample) == false)
            {
                continue;
            }

            if (filtered_global_history::read(history, static_cast<std::uint8_t>(index + 1U), older_sample) == false)
            {
                continue;
            }

            speeds_out[speed_count] = calculate_speed_um_per_ms(older_sample, newer_sample);
            speed_count++;
        }

        return speed_count;
    }

    std::uint16_t calculate_speed_confidence(const filtered_global_history::state &history, const filtered_global::sample &raw_sample)
    {
        std::int64_t speeds[filtered_global_tuning::history_size] = {};
        const std::uint8_t speed_count = collect_history_speeds(history, speeds, filtered_global_tuning::history_size);

        if (speed_count < filtered_global_tuning::minimum_speed_count_for_mad)
        {
            return position_sensorfusion_internal::full_confidence;
        }

        filtered_global::sample newest_sample = {};

        if (filtered_global_history::newest(history, newest_sample) == false)
        {
            return position_sensorfusion_internal::full_confidence;
        }

        const std::int64_t raw_speed = calculate_speed_um_per_ms(newest_sample, raw_sample);
        const std::int64_t median_speed = position_sensorfusion_internal::median_value(speeds, speed_count);
        std::int64_t deviations[filtered_global_tuning::history_size] = {};

        for (std::uint8_t index = 0U; index < speed_count; index++)
        {
            deviations[index] = position_sensorfusion_internal::absolute_i64(speeds[index] - median_speed);
        }

        const std::int64_t median_deviation = position_sensorfusion_internal::median_value(deviations, speed_count);
        const std::int64_t allowed_deviation = (median_deviation * 3) + filtered_global_tuning::speed_mad_margin_um_per_ms;
        const std::int64_t raw_deviation = position_sensorfusion_internal::absolute_i64(raw_speed - median_speed);
        const std::int64_t zero_confidence_deviation = allowed_deviation + filtered_global_tuning::speed_mad_margin_um_per_ms;

        return position_sensorfusion_internal::range_to_confidence(raw_deviation, allowed_deviation, zero_confidence_deviation);
    }
}

namespace history_filter
{
    std::uint16_t calculate_history_confidence(const filtered_global_history::state &history, const filtered_global::sample &raw_sample)
    {
        const std::uint16_t physical_confidence = calculate_physical_confidence(history, raw_sample);
        const std::uint16_t prediction_confidence = calculate_prediction_confidence(history, raw_sample);
        const std::uint16_t speed_confidence = calculate_speed_confidence(history, raw_sample);

        return position_sensorfusion_internal::geometric_mean_confidence(
            physical_confidence,
            prediction_confidence,
            speed_confidence);
    }
}
