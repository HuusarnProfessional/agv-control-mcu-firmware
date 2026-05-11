#include "global_position_history_gates.hpp"

#include <cmath>
#include <cstdint>

namespace
{
    constexpr std::uint16_t full_confidence = 1000U;

    constexpr std::int64_t max_robot_speed_um_per_ms = 1000;
    constexpr std::int64_t physical_jump_margin_um = 200000;

    constexpr std::int64_t prediction_good_residual_um = 100000;
    constexpr std::int64_t prediction_zero_residual_um = 600000;

    constexpr std::int64_t hampel_speed_margin_um_per_ms = 500;
    constexpr std::uint8_t minimum_speeds_for_hampel = 3U;

    std::uint16_t smaller_confidence(std::uint16_t left, std::uint16_t right)
    {
        if (left < right)
        {
            return left;
        }

        return right;
    }

    std::int64_t absolute_i64(std::int64_t value)
    {
        if (value < 0)
        {
            return -value;
        }

        return value;
    }

    std::uint16_t residual_to_confidence(std::int64_t residual_um, std::int64_t good_residual_um, std::int64_t zero_residual_um)
    {
        if (residual_um <= good_residual_um)
        {
            return full_confidence;
        }

        if (residual_um >= zero_residual_um)
        {
            return 0U;
        }

        const std::int64_t range_um = zero_residual_um - good_residual_um;
        const std::int64_t remaining_um = zero_residual_um - residual_um;
        const std::int64_t confidence = remaining_um * full_confidence / range_um;

        return static_cast<std::uint16_t>(confidence);
    }

    std::int64_t calculate_speed_um_per_ms(const global_position_history::sample &older_sample, const global_position_history::sample &newer_sample)
    {
        const std::uint32_t time_delta_ms = global_position_history::calculate_time_delta_ms(older_sample, newer_sample);

        if (time_delta_ms == 0U)
        {
            return 0;
        }

        const std::int64_t distance_um = global_position_history::calculate_distance_um(older_sample, newer_sample);
        const std::int64_t speed_um_per_ms = distance_um / static_cast<std::int64_t>(time_delta_ms);

        return speed_um_per_ms;
    }

    void sort_values(std::int64_t *values, std::uint8_t count)
    {
        for (std::uint8_t outer_index = 0U; outer_index < count; outer_index++)
        {
            for (std::uint8_t inner_index = outer_index + 1U; inner_index < count; inner_index++)
            {
                if (values[inner_index] < values[outer_index])
                {
                    const std::int64_t temporary = values[outer_index];
                    values[outer_index] = values[inner_index];
                    values[inner_index] = temporary;
                }
            }
        }
    }

    std::int64_t median_value(std::int64_t *values, std::uint8_t count)
    {
        if (count == 0U)
        {
            return 0;
        }

        sort_values(values, count);

        const std::uint8_t middle_index = count / 2U;

        if ((count % 2U) == 1U)
        {
            return values[middle_index];
        }

        const std::int64_t left_value = values[middle_index - 1U];
        const std::int64_t right_value = values[middle_index];
        const std::int64_t median = (left_value + right_value) / 2;

        return median;
    }

    std::uint8_t collect_history_speeds(const global_position_history::history_state &history, std::int64_t *speeds_out, std::uint8_t max_count)
    {
        std::uint8_t speed_count = 0U;

        if (history.count < 2U)
        {
            return speed_count;
        }

        for (std::uint8_t index = 0U; index + 1U < history.count; index++)
        {
            if (speed_count >= max_count)
            {
                return speed_count;
            }

            const global_position_history::sample newer_sample = history.samples[index];
            const global_position_history::sample older_sample = history.samples[index + 1U];

            if (newer_sample.valid == false)
            {
                continue;
            }

            if (older_sample.valid == false)
            {
                continue;
            }

            speeds_out[speed_count] = calculate_speed_um_per_ms(older_sample, newer_sample);
            speed_count++;
        }

        return speed_count;
    }
}

namespace global_position_history_gates
{
    std::uint16_t physical_jump_confidence(const global_position_history::history_state &history, const global_position_history::sample &new_sample)
    {
        if (global_position_history::has_last_sample(history) == false)
        {
            return full_confidence;
        }

        const global_position_history::sample last_sample = global_position_history::get_last_sample(history);
        const std::uint32_t time_delta_ms = global_position_history::calculate_time_delta_ms(last_sample, new_sample);

        if (time_delta_ms == 0U)
        {
            return 0U;
        }

        const std::int64_t distance_um = global_position_history::calculate_distance_um(last_sample, new_sample);
        const std::int64_t allowed_distance_um = (max_robot_speed_um_per_ms * static_cast<std::int64_t>(time_delta_ms)) + physical_jump_margin_um;

        if (distance_um <= allowed_distance_um)
        {
            return full_confidence;
        }

        const std::int64_t zero_confidence_distance_um = allowed_distance_um + physical_jump_margin_um;

        return residual_to_confidence(distance_um, allowed_distance_um, zero_confidence_distance_um);
    }

    std::uint16_t prediction_confidence(const global_position_history::history_state &history, const global_position_history::sample &new_sample)
    {
        if (global_position_history::has_two_samples(history) == false)
        {
            return full_confidence;
        }

        const global_position_history::sample last_sample = global_position_history::get_last_sample(history);
        const global_position_history::sample previous_sample = global_position_history::get_previous_sample(history);

        const std::uint32_t previous_time_delta_ms = global_position_history::calculate_time_delta_ms(previous_sample, last_sample);
        const std::uint32_t new_time_delta_ms = global_position_history::calculate_time_delta_ms(last_sample, new_sample);

        if (previous_time_delta_ms == 0U)
        {
            return full_confidence;
        }

        if (new_time_delta_ms == 0U)
        {
            return 0U;
        }

        const std::int64_t previous_delta_x_um = last_sample.x_um - previous_sample.x_um;
        const std::int64_t previous_delta_y_um = last_sample.y_um - previous_sample.y_um;

        const double velocity_x_um_per_ms = static_cast<double>(previous_delta_x_um) / static_cast<double>(previous_time_delta_ms);
        const double velocity_y_um_per_ms = static_cast<double>(previous_delta_y_um) / static_cast<double>(previous_time_delta_ms);

        const double predicted_x_um = static_cast<double>(last_sample.x_um) + (velocity_x_um_per_ms * static_cast<double>(new_time_delta_ms));
        const double predicted_y_um = static_cast<double>(last_sample.y_um) + (velocity_y_um_per_ms * static_cast<double>(new_time_delta_ms));

        const double residual_x_um = static_cast<double>(new_sample.x_um) - predicted_x_um;
        const double residual_y_um = static_cast<double>(new_sample.y_um) - predicted_y_um;
        const double residual_um_double = std::sqrt((residual_x_um * residual_x_um) + (residual_y_um * residual_y_um));
        const std::int64_t residual_um = static_cast<std::int64_t>(residual_um_double);

        return residual_to_confidence(residual_um, prediction_good_residual_um, prediction_zero_residual_um);
    }

    std::uint16_t hampel_speed_confidence(const global_position_history::history_state &history, const global_position_history::sample &new_sample)
    {
        std::int64_t speeds[global_position_history::history_size] = {};
        std::uint8_t speed_count = collect_history_speeds(history, speeds, global_position_history::history_size);

        if (speed_count < minimum_speeds_for_hampel)
        {
            return full_confidence;
        }

        const global_position_history::sample last_sample = global_position_history::get_last_sample(history);
        const std::int64_t new_speed = calculate_speed_um_per_ms(last_sample, new_sample);
        const std::int64_t median_speed = median_value(speeds, speed_count);

        std::int64_t deviations[global_position_history::history_size] = {};

        for (std::uint8_t index = 0U; index < speed_count; index++)
        {
            deviations[index] = absolute_i64(speeds[index] - median_speed);
        }

        const std::int64_t median_deviation = median_value(deviations, speed_count);
        const std::int64_t allowed_deviation = (median_deviation * 3) + hampel_speed_margin_um_per_ms;
        const std::int64_t new_deviation = absolute_i64(new_speed - median_speed);

        if (new_deviation <= allowed_deviation)
        {
            return full_confidence;
        }

        const std::int64_t zero_deviation = allowed_deviation + hampel_speed_margin_um_per_ms;

        return residual_to_confidence(new_deviation, allowed_deviation, zero_deviation);
    }

    std::uint16_t combine_gate_confidence(std::uint16_t physical_confidence, std::uint16_t prediction_confidence, std::uint16_t hampel_confidence)
    {
        std::uint16_t combined = physical_confidence;
        combined = smaller_confidence(combined, prediction_confidence);
        combined = smaller_confidence(combined, hampel_confidence);

        return combined;
    }
}
