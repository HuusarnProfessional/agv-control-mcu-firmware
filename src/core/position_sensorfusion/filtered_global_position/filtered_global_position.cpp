#include "filtered_global_position.hpp"

#include "../../global_positioning/global_position_api.hpp"

#include <cmath>
#include <cstdint>

namespace
{
    constexpr std::uint16_t full_confidence = 1000U;
    constexpr std::uint16_t minimum_initial_confidence = 150U;
    constexpr std::uint16_t minimum_accepted_confidence = 120U;
    constexpr std::uint16_t minimum_tracking_confidence = 80U;

    constexpr std::uint8_t bootstrap_size = 6U;
    constexpr std::uint8_t minimum_bootstrap_sample_count = 3U;
    constexpr std::int64_t bootstrap_good_spread_um = 250000;
    constexpr std::int64_t bootstrap_zero_spread_um = 1000000;

    constexpr std::uint8_t history_size = 64U;
    constexpr std::uint8_t minimum_heading_sample_count = 6U;
    constexpr std::uint8_t full_heading_sample_count = 12U;

    constexpr std::int32_t pi_urad = 3141593;
    constexpr std::int32_t two_pi_urad = 6283185;

    constexpr std::int64_t max_robot_speed_um_per_ms = 1500;
    constexpr std::int64_t physical_jump_margin_um = 500000;
    constexpr std::int64_t prediction_good_residual_um = 150000;
    constexpr std::int64_t prediction_zero_residual_um = 1200000;
    constexpr std::int64_t hampel_speed_margin_um_per_ms = 700;
    constexpr std::uint8_t minimum_speeds_for_hampel = 3U;

    constexpr std::int64_t position_step_base_um = 30000;
    constexpr std::int64_t position_step_speed_um_per_ms = 300;

    constexpr std::int64_t heading_min_distance_um = 500000;
    constexpr std::int64_t heading_full_distance_um = 1500000;
    constexpr std::int64_t heading_good_line_error_um = 120000;
    constexpr std::int64_t heading_zero_line_error_um = 450000;
    constexpr std::uint32_t heading_max_window_age_ms = 6000U;

    constexpr std::uint32_t position_confidence_full_age_ms = 250U;
    constexpr std::uint32_t position_confidence_zero_age_ms = 2500U;
    constexpr std::uint32_t heading_confidence_full_age_ms = 500U;
    constexpr std::uint32_t heading_confidence_zero_age_ms = 5000U;

    struct accepted_sample
    {
        bool valid = false;
        std::uint32_t sample_id = 0U;
        std::uint32_t request_id = 0U;
        std::uint32_t received_time_ms = 0U;
        std::int64_t x_um = 0;
        std::int64_t y_um = 0;
        std::int64_t z_um = 0;
        std::uint16_t confidence_position = 0U;
    };

    struct stable_state
    {
        bool has_position = false;
        std::int64_t x_um = 0;
        std::int64_t y_um = 0;
        std::int64_t z_um = 0;
        std::uint16_t confidence_position = 0U;
        std::uint32_t position_time_ms = 0U;

        bool has_heading = false;
        std::int32_t heading_urad = 0;
        std::uint16_t confidence_heading = 0U;
        std::uint32_t heading_time_ms = 0U;
        std::uint8_t heading_sample_count = 0U;
        std::int64_t heading_distance_um = 0;
    };

    accepted_sample history[history_size] = {};
    accepted_sample bootstrap_samples[bootstrap_size] = {};
    std::uint8_t history_count = 0U;
    std::uint8_t bootstrap_count = 0U;
    stable_state stable = {};
    filtered_global_position::output_snapshot latest_output = {};
    bool has_last_sample_id = false;
    std::uint32_t last_sample_id = 0U;

    std::uint32_t get_age_ms(std::uint32_t now_ms, std::uint32_t start_time_ms)
    {
        if (now_ms < start_time_ms)
        {
            return 0U;
        }

        return now_ms - start_time_ms;
    }

    std::int64_t absolute_i64(std::int64_t value)
    {
        if (value < 0)
        {
            return -value;
        }

        return value;
    }

    std::int32_t normalize_angle_urad(std::int32_t angle_urad)
    {
        const double normalized_double = std::remainder(static_cast<double>(angle_urad), static_cast<double>(two_pi_urad));
        std::int32_t normalized = static_cast<std::int32_t>(normalized_double);

        if (normalized > pi_urad)
        {
            normalized -= two_pi_urad;
        }

        if (normalized < -pi_urad)
        {
            normalized += two_pi_urad;
        }

        return normalized;
    }

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
        const std::uint32_t scaled = multiplied / full_confidence;

        return static_cast<std::uint16_t>(scaled);
    }

    std::uint16_t quality_factor_to_confidence(std::uint8_t quality_factor)
    {
        const std::uint16_t scaled = static_cast<std::uint16_t>(quality_factor) * 10U;

        if (scaled > full_confidence)
        {
            return full_confidence;
        }

        return scaled;
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
        const std::uint32_t age_over_full_ms = age_ms - full_age_ms;
        const std::uint32_t remaining_ms = range_ms - age_over_full_ms;
        const std::uint32_t confidence = remaining_ms * full_confidence / range_ms;

        return static_cast<std::uint16_t>(confidence);
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

    std::uint16_t sample_count_to_confidence(std::uint8_t sample_count)
    {
        if (sample_count < minimum_heading_sample_count)
        {
            return 0U;
        }

        if (sample_count >= full_heading_sample_count)
        {
            return full_confidence;
        }

        const std::uint8_t range = full_heading_sample_count - minimum_heading_sample_count;
        const std::uint8_t progress = sample_count - minimum_heading_sample_count;
        const std::uint16_t confidence = static_cast<std::uint16_t>(progress) * full_confidence / static_cast<std::uint16_t>(range);

        return confidence;
    }

    std::int64_t calculate_distance_um(std::int64_t delta_x_um, std::int64_t delta_y_um)
    {
        const double delta_x = static_cast<double>(delta_x_um);
        const double delta_y = static_cast<double>(delta_y_um);
        const double distance = std::sqrt((delta_x * delta_x) + (delta_y * delta_y));

        return static_cast<std::int64_t>(distance);
    }

    std::int64_t calculate_sample_distance_um(const accepted_sample &older_sample, const accepted_sample &newer_sample)
    {
        const std::int64_t delta_x_um = newer_sample.x_um - older_sample.x_um;
        const std::int64_t delta_y_um = newer_sample.y_um - older_sample.y_um;

        return calculate_distance_um(delta_x_um, delta_y_um);
    }

    std::uint32_t calculate_time_delta_ms(const accepted_sample &older_sample, const accepted_sample &newer_sample)
    {
        if (newer_sample.received_time_ms < older_sample.received_time_ms)
        {
            return 0U;
        }

        return newer_sample.received_time_ms - older_sample.received_time_ms;
    }

    std::int64_t calculate_speed_um_per_ms(const accepted_sample &older_sample, const accepted_sample &newer_sample)
    {
        const std::uint32_t time_delta_ms = calculate_time_delta_ms(older_sample, newer_sample);

        if (time_delta_ms == 0U)
        {
            return 0;
        }

        const std::int64_t distance_um = calculate_sample_distance_um(older_sample, newer_sample);
        const std::int64_t speed_um_per_ms = distance_um / static_cast<std::int64_t>(time_delta_ms);

        return speed_um_per_ms;
    }

    void sort_values(std::int64_t *values, std::uint8_t count)
    {
        for (std::uint8_t outer_index = 0U; outer_index < count; outer_index++)
        {
            for (std::uint8_t inner_index = static_cast<std::uint8_t>(outer_index + 1U); inner_index < count; inner_index++)
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

    accepted_sample convert_sample(const global_position_api::global_position_sample &sample)
    {
        accepted_sample converted = {};

        converted.valid = true;
        converted.sample_id = sample.sample_id;
        converted.request_id = sample.request_id;
        converted.received_time_ms = sample.received_time_ms;
        converted.x_um = static_cast<std::int64_t>(sample.x_mm) * 1000;
        converted.y_um = static_cast<std::int64_t>(sample.y_mm) * 1000;
        converted.z_um = static_cast<std::int64_t>(sample.z_mm) * 1000;
        converted.confidence_position = quality_factor_to_confidence(sample.quality_factor);

        return converted;
    }

    bool api_sample_is_valid(const global_position_api::global_position_sample &sample)
    {
        if (sample.valid == false)
        {
            return false;
        }

        if (sample.status != global_position_api::global_position_status::ok)
        {
            return false;
        }

        return true;
    }

    bool sample_is_new(const global_position_api::global_position_sample &sample)
    {
        if (has_last_sample_id == false)
        {
            return true;
        }

        if (sample.sample_id == last_sample_id)
        {
            return false;
        }

        return true;
    }

    void push_history(const accepted_sample &sample)
    {
        if (history_count > 0U)
        {
            for (std::uint8_t index = history_count; index > 0U; index--)
            {
                if (index >= history_size)
                {
                    continue;
                }

                history[index] = history[index - 1U];
            }
        }

        history[0U] = sample;

        if (history_count < history_size)
        {
            history_count++;
        }
    }

    void clear_bootstrap()
    {
        for (std::uint8_t index = 0U; index < bootstrap_size; index++)
        {
            bootstrap_samples[index] = {};
        }

        bootstrap_count = 0U;
    }

    void push_bootstrap_sample(const accepted_sample &sample)
    {
        if (bootstrap_count > 0U)
        {
            for (std::uint8_t index = bootstrap_count; index > 0U; index--)
            {
                if (index >= bootstrap_size)
                {
                    continue;
                }

                bootstrap_samples[index] = bootstrap_samples[index - 1U];
            }
        }

        bootstrap_samples[0U] = sample;

        if (bootstrap_count < bootstrap_size)
        {
            bootstrap_count++;
        }
    }

    bool calculate_bootstrap_center(accepted_sample &sample_out)
    {
        if (bootstrap_count < minimum_bootstrap_sample_count)
        {
            return false;
        }

        std::int64_t weighted_x_um = 0;
        std::int64_t weighted_y_um = 0;
        std::int64_t weighted_z_um = 0;
        std::uint32_t confidence_sum = 0U;

        for (std::uint8_t index = 0U; index < bootstrap_count; index++)
        {
            const std::uint32_t confidence = bootstrap_samples[index].confidence_position;

            weighted_x_um += bootstrap_samples[index].x_um * static_cast<std::int64_t>(confidence);
            weighted_y_um += bootstrap_samples[index].y_um * static_cast<std::int64_t>(confidence);
            weighted_z_um += bootstrap_samples[index].z_um * static_cast<std::int64_t>(confidence);
            confidence_sum += confidence;
        }

        if (confidence_sum == 0U)
        {
            return false;
        }

        sample_out = bootstrap_samples[0U];
        sample_out.x_um = weighted_x_um / static_cast<std::int64_t>(confidence_sum);
        sample_out.y_um = weighted_y_um / static_cast<std::int64_t>(confidence_sum);
        sample_out.z_um = weighted_z_um / static_cast<std::int64_t>(confidence_sum);
        sample_out.confidence_position = static_cast<std::uint16_t>(confidence_sum / static_cast<std::uint32_t>(bootstrap_count));

        return true;
    }

    std::int64_t calculate_bootstrap_max_spread_um(const accepted_sample &center_sample)
    {
        std::int64_t max_spread_um = 0;

        for (std::uint8_t index = 0U; index < bootstrap_count; index++)
        {
            const std::int64_t delta_x_um = bootstrap_samples[index].x_um - center_sample.x_um;
            const std::int64_t delta_y_um = bootstrap_samples[index].y_um - center_sample.y_um;
            const std::int64_t spread_um = calculate_distance_um(delta_x_um, delta_y_um);

            if (spread_um > max_spread_um)
            {
                max_spread_um = spread_um;
            }
        }

        return max_spread_um;
    }

    bool try_build_initial_sample_from_bootstrap(accepted_sample &sample_out, std::uint16_t &history_confidence_out)
    {
        accepted_sample center_sample = {};
        const bool has_center = calculate_bootstrap_center(center_sample);

        if (has_center == false)
        {
            history_confidence_out = 0U;
            return false;
        }

        const std::int64_t max_spread_um = calculate_bootstrap_max_spread_um(center_sample);
        history_confidence_out = range_to_confidence(max_spread_um, bootstrap_good_spread_um, bootstrap_zero_spread_um);
        center_sample.confidence_position = multiply_confidence(center_sample.confidence_position, history_confidence_out);

        if (center_sample.confidence_position < minimum_initial_confidence)
        {
            return false;
        }

        sample_out = center_sample;
        return true;
    }

    std::uint16_t physical_jump_confidence(const accepted_sample &new_sample)
    {
        if (history_count == 0U)
        {
            return full_confidence;
        }

        const accepted_sample last_sample = history[0U];
        const std::uint32_t time_delta_ms = calculate_time_delta_ms(last_sample, new_sample);

        if (time_delta_ms == 0U)
        {
            return 0U;
        }

        const std::int64_t distance_um = calculate_sample_distance_um(last_sample, new_sample);
        const std::int64_t allowed_distance_um = (max_robot_speed_um_per_ms * static_cast<std::int64_t>(time_delta_ms)) + physical_jump_margin_um;
        const std::int64_t zero_confidence_distance_um = allowed_distance_um + physical_jump_margin_um;

        return range_to_confidence(distance_um, allowed_distance_um, zero_confidence_distance_um);
    }

    std::uint16_t prediction_confidence(const accepted_sample &new_sample)
    {
        if (history_count < 2U)
        {
            return full_confidence;
        }

        const accepted_sample last_sample = history[0U];
        const accepted_sample previous_sample = history[1U];
        const std::uint32_t previous_time_delta_ms = calculate_time_delta_ms(previous_sample, last_sample);
        const std::uint32_t new_time_delta_ms = calculate_time_delta_ms(last_sample, new_sample);

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

        return range_to_confidence(residual_um, prediction_good_residual_um, prediction_zero_residual_um);
    }

    std::uint8_t collect_history_speeds(std::int64_t *speeds_out, std::uint8_t max_count)
    {
        std::uint8_t speed_count = 0U;

        if (history_count < 2U)
        {
            return speed_count;
        }

        for (std::uint8_t index = 0U; index + 1U < history_count; index++)
        {
            if (speed_count >= max_count)
            {
                return speed_count;
            }

            const accepted_sample newer_sample = history[index];
            const accepted_sample older_sample = history[index + 1U];

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

    std::uint16_t hampel_speed_confidence(const accepted_sample &new_sample)
    {
        std::int64_t speeds[history_size] = {};
        const std::uint8_t speed_count = collect_history_speeds(speeds, history_size);

        if (speed_count < minimum_speeds_for_hampel)
        {
            return full_confidence;
        }

        const accepted_sample last_sample = history[0U];
        const std::int64_t new_speed = calculate_speed_um_per_ms(last_sample, new_sample);
        const std::int64_t median_speed = median_value(speeds, speed_count);
        std::int64_t deviations[history_size] = {};

        for (std::uint8_t index = 0U; index < speed_count; index++)
        {
            deviations[index] = absolute_i64(speeds[index] - median_speed);
        }

        const std::int64_t median_deviation = median_value(deviations, speed_count);
        const std::int64_t allowed_deviation = (median_deviation * 3) + hampel_speed_margin_um_per_ms;
        const std::int64_t new_deviation = absolute_i64(new_speed - median_speed);
        const std::int64_t zero_deviation = allowed_deviation + hampel_speed_margin_um_per_ms;

        return range_to_confidence(new_deviation, allowed_deviation, zero_deviation);
    }

    std::uint16_t calculate_history_confidence(const accepted_sample &new_sample)
    {
        const std::uint16_t physical_confidence = physical_jump_confidence(new_sample);
        const std::uint16_t predicted_confidence = prediction_confidence(new_sample);
        const std::uint16_t speed_confidence = hampel_speed_confidence(new_sample);
        std::uint16_t combined_confidence = physical_confidence;

        combined_confidence = smaller_confidence(combined_confidence, predicted_confidence);
        combined_confidence = smaller_confidence(combined_confidence, speed_confidence);

        return combined_confidence;
    }

    std::int64_t calculate_max_position_step_um(std::uint32_t received_time_ms)
    {
        if (stable.has_position == false)
        {
            return 0;
        }

        const std::uint32_t time_delta_ms = get_age_ms(received_time_ms, stable.position_time_ms);
        const std::int64_t step_um = position_step_base_um + (position_step_speed_um_per_ms * static_cast<std::int64_t>(time_delta_ms));

        return step_um;
    }

    void limit_step_vector(std::int64_t &step_x_um, std::int64_t &step_y_um, std::int64_t max_step_um)
    {
        if (max_step_um <= 0)
        {
            step_x_um = 0;
            step_y_um = 0;
            return;
        }

        const std::int64_t step_distance_um = calculate_distance_um(step_x_um, step_y_um);

        if (step_distance_um <= max_step_um)
        {
            return;
        }

        if (step_distance_um == 0)
        {
            return;
        }

        step_x_um = step_x_um * max_step_um / step_distance_um;
        step_y_um = step_y_um * max_step_um / step_distance_um;
    }

    std::int64_t weighted_step_um(std::int64_t difference_um, std::uint16_t current_confidence, std::uint16_t sample_confidence)
    {
        const std::uint32_t total_confidence = static_cast<std::uint32_t>(current_confidence) + static_cast<std::uint32_t>(sample_confidence);

        if (total_confidence == 0U)
        {
            return 0;
        }

        const std::int64_t weighted_difference = difference_um * static_cast<std::int64_t>(sample_confidence);
        const std::int64_t step_um = weighted_difference / static_cast<std::int64_t>(total_confidence);

        return step_um;
    }

    std::int32_t weighted_heading_step_urad(std::int32_t difference_urad, std::uint16_t current_confidence, std::uint16_t sample_confidence)
    {
        const std::uint32_t total_confidence = static_cast<std::uint32_t>(current_confidence) + static_cast<std::uint32_t>(sample_confidence);

        if (total_confidence == 0U)
        {
            return 0;
        }

        const std::int64_t weighted_difference = static_cast<std::int64_t>(difference_urad) * static_cast<std::int64_t>(sample_confidence);
        const std::int64_t step_urad = weighted_difference / static_cast<std::int64_t>(total_confidence);

        return static_cast<std::int32_t>(step_urad);
    }

    std::int64_t calculate_line_error_um(const accepted_sample &line_start, const accepted_sample &line_end, const accepted_sample &sample)
    {
        const std::int64_t line_delta_x_um = line_end.x_um - line_start.x_um;
        const std::int64_t line_delta_y_um = line_end.y_um - line_start.y_um;
        const std::int64_t point_delta_x_um = sample.x_um - line_start.x_um;
        const std::int64_t point_delta_y_um = sample.y_um - line_start.y_um;
        const double line_length = static_cast<double>(calculate_distance_um(line_delta_x_um, line_delta_y_um));

        if (line_length <= 0.0)
        {
            return 0;
        }

        const double cross = static_cast<double>(line_delta_x_um) * static_cast<double>(point_delta_y_um) - static_cast<double>(line_delta_y_um) * static_cast<double>(point_delta_x_um);
        const double error_um = std::fabs(cross) / line_length;

        return static_cast<std::int64_t>(error_um);
    }

    std::uint16_t calculate_window_position_confidence(std::uint8_t last_index)
    {
        std::uint16_t confidence = full_confidence;

        for (std::uint8_t index = 0U; index <= last_index; index++)
        {
            confidence = smaller_confidence(confidence, history[index].confidence_position);
        }

        return confidence;
    }

    std::int64_t calculate_max_line_error_um(std::uint8_t last_index)
    {
        const accepted_sample newest_sample = history[0U];
        const accepted_sample oldest_sample = history[last_index];
        std::int64_t max_error_um = 0;

        for (std::uint8_t index = 1U; index < last_index; index++)
        {
            const std::int64_t error_um = calculate_line_error_um(oldest_sample, newest_sample, history[index]);

            if (error_um > max_error_um)
            {
                max_error_um = error_um;
            }
        }

        return max_error_um;
    }

    bool find_heading_window(std::uint8_t &oldest_index_out, std::int64_t &distance_um_out)
    {
        if (history_count < minimum_heading_sample_count)
        {
            return false;
        }

        const accepted_sample newest_sample = history[0U];
        std::uint8_t best_index = 0U;
        std::int64_t best_distance_um = 0;

        for (std::uint8_t index = 1U; index < history_count; index++)
        {
            const accepted_sample older_sample = history[index];
            const std::uint32_t age_ms = calculate_time_delta_ms(older_sample, newest_sample);

            if (age_ms > heading_max_window_age_ms)
            {
                continue;
            }

            const std::int64_t distance_um = calculate_sample_distance_um(older_sample, newest_sample);

            if (distance_um > best_distance_um)
            {
                best_index = index;
                best_distance_um = distance_um;
            }
        }

        if (best_distance_um < heading_min_distance_um)
        {
            return false;
        }

        oldest_index_out = best_index;
        distance_um_out = best_distance_um;

        return true;
    }

    void update_heading_from_history()
    {
        std::uint8_t oldest_index = 0U;
        std::int64_t heading_distance_um = 0;
        const bool has_window = find_heading_window(oldest_index, heading_distance_um);

        if (has_window == false)
        {
            return;
        }

        const accepted_sample newest_sample = history[0U];
        const accepted_sample oldest_sample = history[oldest_index];
        const std::int64_t delta_x_um = newest_sample.x_um - oldest_sample.x_um;
        const std::int64_t delta_y_um = newest_sample.y_um - oldest_sample.y_um;
        const double heading_rad = std::atan2(static_cast<double>(delta_y_um), static_cast<double>(delta_x_um));
        const std::int32_t measured_heading_urad = normalize_angle_urad(static_cast<std::int32_t>(std::llround(heading_rad * 1000000.0)));
        const std::uint8_t sample_count = static_cast<std::uint8_t>(oldest_index + 1U);
        const std::int64_t max_line_error_um = calculate_max_line_error_um(oldest_index);
        const std::uint16_t distance_confidence = growth_to_confidence(heading_distance_um, heading_min_distance_um, heading_full_distance_um);
        const std::uint16_t count_confidence = sample_count_to_confidence(sample_count);
        const std::uint16_t line_confidence = range_to_confidence(max_line_error_um, heading_good_line_error_um, heading_zero_line_error_um);
        const std::uint16_t position_confidence = calculate_window_position_confidence(oldest_index);
        std::uint16_t measured_confidence = distance_confidence;

        measured_confidence = smaller_confidence(measured_confidence, count_confidence);
        measured_confidence = smaller_confidence(measured_confidence, line_confidence);
        measured_confidence = smaller_confidence(measured_confidence, position_confidence);

        if (measured_confidence < minimum_accepted_confidence)
        {
            return;
        }

        if (stable.has_heading == false)
        {
            stable.has_heading = true;
            stable.heading_urad = measured_heading_urad;
            stable.confidence_heading = measured_confidence;
            stable.heading_time_ms = newest_sample.received_time_ms;
            stable.heading_sample_count = sample_count;
            stable.heading_distance_um = heading_distance_um;
            return;
        }

        const std::int32_t difference_urad = normalize_angle_urad(measured_heading_urad - stable.heading_urad);
        const std::int32_t step_urad = weighted_heading_step_urad(difference_urad, stable.confidence_heading, measured_confidence);

        stable.heading_urad = normalize_angle_urad(stable.heading_urad + step_urad);
        stable.confidence_heading = larger_confidence(stable.confidence_heading, measured_confidence);
        stable.heading_time_ms = newest_sample.received_time_ms;
        stable.heading_sample_count = sample_count;
        stable.heading_distance_um = heading_distance_um;
    }

    void accept_initial_sample(const accepted_sample &sample)
    {
        stable.has_position = true;
        stable.x_um = sample.x_um;
        stable.y_um = sample.y_um;
        stable.z_um = sample.z_um;
        stable.confidence_position = sample.confidence_position;
        stable.position_time_ms = sample.received_time_ms;
        push_history(sample);
        clear_bootstrap();
    }

    void accept_sample(const accepted_sample &sample, std::uint16_t filtered_confidence)
    {
        if (stable.has_position == false)
        {
            accept_initial_sample(sample);
            return;
        }

        std::int64_t step_x_um = weighted_step_um(sample.x_um - stable.x_um, stable.confidence_position, filtered_confidence);
        std::int64_t step_y_um = weighted_step_um(sample.y_um - stable.y_um, stable.confidence_position, filtered_confidence);
        const std::int64_t max_step_um = calculate_max_position_step_um(sample.received_time_ms);

        limit_step_vector(step_x_um, step_y_um, max_step_um);

        stable.x_um += step_x_um;
        stable.y_um += step_y_um;
        stable.z_um += weighted_step_um(sample.z_um - stable.z_um, stable.confidence_position, filtered_confidence);
        stable.confidence_position = larger_confidence(stable.confidence_position, filtered_confidence);
        stable.position_time_ms = sample.received_time_ms;

        accepted_sample smoothed_sample = sample;
        smoothed_sample.x_um = stable.x_um;
        smoothed_sample.y_um = stable.y_um;
        smoothed_sample.z_um = stable.z_um;
        smoothed_sample.confidence_position = filtered_confidence;
        push_history(smoothed_sample);
        update_heading_from_history();
    }

    filtered_global_position::output_snapshot build_output(std::uint32_t now_ms, bool is_new_sample, bool accepted, bool rejected, const accepted_sample &sample, std::uint16_t raw_confidence, std::uint16_t history_confidence)
    {
        filtered_global_position::output_snapshot output = {};

        output.is_new_sample = is_new_sample;
        output.accepted = accepted;
        output.rejected = rejected;
        output.raw_confidence_position = raw_confidence;
        output.history_confidence = history_confidence;
        output.accepted_sample_count = history_count;
        output.sample_id = sample.sample_id;
        output.request_id = sample.request_id;
        output.received_time_ms = sample.received_time_ms;

        if (stable.has_position == true)
        {
            const std::uint32_t age_ms = get_age_ms(now_ms, stable.position_time_ms);
            const std::uint16_t age_confidence = age_to_confidence(age_ms, position_confidence_full_age_ms, position_confidence_zero_age_ms);

            output.has_position = true;
            output.x_um = stable.x_um;
            output.y_um = stable.y_um;
            output.z_um = stable.z_um;
            output.confidence_position = multiply_confidence(stable.confidence_position, age_confidence);
        }

        if (stable.has_heading == true)
        {
            const std::uint32_t age_ms = get_age_ms(now_ms, stable.heading_time_ms);
            const std::uint16_t age_confidence = age_to_confidence(age_ms, heading_confidence_full_age_ms, heading_confidence_zero_age_ms);

            output.has_heading = true;
            output.heading_urad = stable.heading_urad;
            output.confidence_heading = multiply_confidence(stable.confidence_heading, age_confidence);
            output.heading_sample_count = stable.heading_sample_count;
            output.heading_distance_um = stable.heading_distance_um;
        }

        latest_output = output;
        return latest_output;
    }

    filtered_global_position::output_snapshot reject_sample(std::uint32_t now_ms, const accepted_sample &sample, std::uint16_t raw_confidence, std::uint16_t history_confidence)
    {
        return build_output(now_ms, true, false, true, sample, raw_confidence, history_confidence);
    }

    filtered_global_position::output_snapshot process_new_sample(std::uint32_t now_ms, const global_position_api::global_position_sample &api_sample)
    {
        const accepted_sample sample = convert_sample(api_sample);
        const std::uint16_t raw_confidence = sample.confidence_position;

        last_sample_id = sample.sample_id;
        has_last_sample_id = true;

        if (raw_confidence < minimum_accepted_confidence)
        {
            return reject_sample(now_ms, sample, raw_confidence, 0U);
        }

        const std::uint16_t history_confidence = calculate_history_confidence(sample);
        const std::uint16_t filtered_confidence = multiply_confidence(raw_confidence, history_confidence);

        if (stable.has_position == false)
        {
            if (raw_confidence < minimum_initial_confidence)
            {
                return reject_sample(now_ms, sample, raw_confidence, history_confidence);
            }

            push_bootstrap_sample(sample);

            accepted_sample initial_sample = {};
            std::uint16_t bootstrap_history_confidence = 0U;
            const bool has_initial_sample = try_build_initial_sample_from_bootstrap(initial_sample, bootstrap_history_confidence);

            if (has_initial_sample == false)
            {
                return build_output(now_ms, true, false, false, sample, raw_confidence, bootstrap_history_confidence);
            }

            accept_initial_sample(initial_sample);
            return build_output(now_ms, true, true, false, initial_sample, raw_confidence, bootstrap_history_confidence);
        }

        std::uint16_t tracking_confidence = filtered_confidence;

        if ((tracking_confidence < minimum_tracking_confidence) && (raw_confidence >= minimum_initial_confidence))
        {
            tracking_confidence = minimum_tracking_confidence;
        }

        if (tracking_confidence < minimum_tracking_confidence)
        {
            return reject_sample(now_ms, sample, raw_confidence, history_confidence);
        }

        accept_sample(sample, tracking_confidence);
        return build_output(now_ms, true, true, false, sample, raw_confidence, history_confidence);
    }
}

namespace filtered_global_position
{
    void init()
    {
        for (std::uint8_t index = 0U; index < history_size; index++)
        {
            history[index] = {};
        }

        clear_bootstrap();
        history_count = 0U;
        stable = {};
        latest_output = {};
        has_last_sample_id = false;
        last_sample_id = 0U;
    }

    output_snapshot update(std::uint32_t now_ms)
    {
        global_position_api::global_position_sample api_sample = {};
        const bool has_sample = global_position_api::read_sample(api_sample);

        if (has_sample == false)
        {
            accepted_sample empty_sample = {};
            return build_output(now_ms, false, false, false, empty_sample, 0U, 0U);
        }

        if (api_sample_is_valid(api_sample) == false)
        {
            accepted_sample invalid_sample = {};
            invalid_sample.sample_id = api_sample.sample_id;
            invalid_sample.request_id = api_sample.request_id;
            invalid_sample.received_time_ms = api_sample.received_time_ms;
            return build_output(now_ms, false, false, true, invalid_sample, 0U, 0U);
        }

        if (sample_is_new(api_sample) == false)
        {
            accepted_sample latest_sample = {};
            latest_sample.sample_id = api_sample.sample_id;
            latest_sample.request_id = api_sample.request_id;
            latest_sample.received_time_ms = api_sample.received_time_ms;
            latest_sample.confidence_position = quality_factor_to_confidence(api_sample.quality_factor);
            return build_output(now_ms, false, false, false, latest_sample, latest_sample.confidence_position, latest_output.history_confidence);
        }

        return process_new_sample(now_ms, api_sample);
    }

    output_snapshot read_output(std::uint32_t now_ms)
    {
        accepted_sample latest_sample = {};
        latest_sample.sample_id = latest_output.sample_id;
        latest_sample.request_id = latest_output.request_id;
        latest_sample.received_time_ms = latest_output.received_time_ms;
        latest_sample.confidence_position = latest_output.raw_confidence_position;

        return build_output(now_ms, false, false, false, latest_sample, latest_output.raw_confidence_position, latest_output.history_confidence);
    }
}
