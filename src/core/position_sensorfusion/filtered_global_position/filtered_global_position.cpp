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

    constexpr filtered_global_position::filtered_global_heading_mode global_heading_mode = filtered_global_position::filtered_global_heading_mode::huber_pca;

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

    constexpr std::int64_t chord_heading_min_distance_um = 300000;
    constexpr std::int64_t chord_heading_full_distance_um = 900000;
    constexpr std::int64_t chord_heading_good_line_error_um = 120000;
    constexpr std::int64_t chord_heading_zero_line_error_um = 450000;
    constexpr std::uint32_t chord_heading_max_window_age_ms = 2500U;

    constexpr std::uint32_t huber_pca_heading_max_age_ms = 2000U;
    constexpr std::uint32_t huber_pca_heading_estimated_delay_ms = 900U;
    constexpr std::uint32_t huber_pca_heading_reference_window_ms = 700U;
    constexpr std::int64_t huber_pca_heading_min_distance_um = 300000;
    constexpr std::int64_t huber_pca_heading_full_distance_um = 900000;
    constexpr std::uint8_t huber_pca_heading_min_sample_count = 6U;
    constexpr std::uint8_t huber_pca_heading_full_sample_count = 12U;
    constexpr std::uint8_t huber_pca_heading_max_sample_count = 15U;
    constexpr std::uint32_t huber_pca_delta_um = 20000U;
    constexpr std::uint32_t huber_pca_good_residual_um = 20000U;
    constexpr std::uint32_t huber_pca_zero_residual_um = 120000U;
    constexpr std::uint8_t huber_pca_iteration_count = 6U;
    constexpr std::uint32_t position_anchor_max_age_ms = 1200U;
    constexpr std::uint32_t position_anchor_estimated_delay_ms = 500U;
    constexpr std::uint32_t position_anchor_reference_window_ms = 500U;
    constexpr std::uint8_t position_anchor_min_sample_count = 3U;
    constexpr std::uint8_t position_anchor_full_sample_count = 7U;
    constexpr std::uint8_t position_anchor_max_sample_count = 7U;
    constexpr std::uint32_t position_anchor_huber_delta_um = 80000U;
    constexpr std::uint32_t position_anchor_good_residual_um = 50000U;
    constexpr std::uint32_t position_anchor_zero_residual_um = 300000U;
    constexpr std::uint8_t position_anchor_iteration_count = 5U;
    constexpr std::uint16_t default_candidate_anchor_heading_confidence_gain_permille = 2500U;
    constexpr std::uint16_t default_candidate_position_anchor_confidence_gain_permille = 1200U;
    constexpr std::uint16_t maximum_candidate_anchor_confidence_gain_permille = 10000U;

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
        bool has_local_reference = false;
        std::int64_t local_x_um = 0;
        std::int64_t local_y_um = 0;
        std::int32_t local_heading_urad = 0;
        std::uint16_t pose_id = 0U;
        std::uint8_t branch_id = 0U;
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
        std::uint32_t position_reference_time_ms = 0U;
        std::uint32_t position_reference_sample_id = 0U;
        std::uint16_t position_reference_pose_id = 0U;
        std::uint8_t position_reference_branch_id = 0U;
        std::int64_t position_reference_x_um = 0;
        std::int64_t position_reference_y_um = 0;
        std::int64_t position_reference_z_um = 0;
        bool position_reference_has_local_reference = false;
        std::int64_t position_reference_local_x_um = 0;
        std::int64_t position_reference_local_y_um = 0;
        std::int32_t position_reference_local_heading_urad = 0;
        std::uint8_t position_anchor_sample_count = 0U;
        std::uint32_t position_anchor_median_residual_um = 0U;
        std::uint32_t position_anchor_window_age_ms = 0U;
        std::uint32_t heading_reference_time_ms = 0U;
        std::uint32_t heading_reference_sample_id = 0U;
        std::uint32_t heading_estimated_delay_ms = 0U;
        std::uint16_t heading_reference_pose_id = 0U;
        std::uint8_t heading_reference_branch_id = 0U;
        std::int64_t heading_reference_x_um = 0;
        std::int64_t heading_reference_y_um = 0;
        std::int64_t heading_reference_z_um = 0;
        std::uint32_t heading_fit_residual_um = 0U;
        std::uint16_t candidate_anchor_position_confidence = 0U;
        std::uint16_t candidate_anchor_heading_confidence = 0U;
        std::uint16_t candidate_anchor_adjusted_heading_confidence = 0U;
        std::uint16_t candidate_anchor_confidence = 0U;
        std::uint16_t candidate_position_anchor_confidence = 0U;
        std::uint8_t huber_pca_used_sample_count = 0U;
        std::uint32_t huber_pca_median_residual_um = 0U;
        std::uint32_t huber_pca_max_residual_um = 0U;
        std::uint32_t huber_pca_movement_distance_um = 0U;
        std::uint32_t huber_pca_window_age_ms = 0U;
        std::uint8_t chord_used_sample_count = 0U;
        std::int64_t chord_distance_um = 0;
        std::int64_t chord_max_line_error_um = 0;
        std::uint32_t chord_window_age_ms = 0U;
    };

    struct heading_estimate_candidate
    {
        std::int32_t heading_urad = 0;
        std::uint16_t confidence_heading = 0U;
        std::uint32_t heading_time_ms = 0U;
        std::uint8_t heading_sample_count = 0U;
        std::int64_t heading_distance_um = 0;
        std::uint32_t heading_reference_time_ms = 0U;
        std::uint32_t heading_reference_sample_id = 0U;
        std::uint32_t heading_estimated_delay_ms = 0U;
        std::uint16_t heading_reference_pose_id = 0U;
        std::uint8_t heading_reference_branch_id = 0U;
        std::int64_t heading_reference_x_um = 0;
        std::int64_t heading_reference_y_um = 0;
        std::int64_t heading_reference_z_um = 0;
        std::uint32_t heading_fit_residual_um = 0U;
        std::uint16_t candidate_anchor_position_confidence = 0U;
        std::uint16_t candidate_anchor_heading_confidence = 0U;
        std::uint16_t candidate_anchor_adjusted_heading_confidence = 0U;
        std::uint16_t candidate_anchor_confidence = 0U;
        std::uint8_t huber_pca_used_sample_count = 0U;
        std::uint32_t huber_pca_median_residual_um = 0U;
        std::uint32_t huber_pca_max_residual_um = 0U;
        std::uint32_t huber_pca_movement_distance_um = 0U;
        std::uint32_t huber_pca_window_age_ms = 0U;
        std::uint8_t chord_used_sample_count = 0U;
        std::int64_t chord_distance_um = 0;
        std::int64_t chord_max_line_error_um = 0;
        std::uint32_t chord_window_age_ms = 0U;
    };

    accepted_sample history[history_size] = {};
    accepted_sample bootstrap_samples[bootstrap_size] = {};
    std::uint8_t history_count = 0U;
    std::uint8_t bootstrap_count = 0U;
    stable_state stable = {};
    filtered_global_position::output_snapshot latest_output = {};
    bool has_last_sample_id = false;
    std::uint32_t last_sample_id = 0U;
    std::uint16_t candidate_anchor_heading_confidence_gain_permille = default_candidate_anchor_heading_confidence_gain_permille;
    std::uint16_t candidate_position_anchor_confidence_gain_permille = default_candidate_position_anchor_confidence_gain_permille;

    bool calculate_motion_compensated_anchor_reference(const accepted_sample *samples, std::uint8_t sample_count, const accepted_sample &reference_sample, std::int32_t measured_heading_urad, accepted_sample &center_sample_out, std::uint16_t &reference_position_confidence_out, std::uint32_t &median_residual_um_out);

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

    std::uint16_t apply_confidence_gain(std::uint16_t confidence, std::uint16_t gain_permille)
    {
        const std::uint32_t gained_confidence = static_cast<std::uint32_t>(confidence) * static_cast<std::uint32_t>(gain_permille) / 1000U;

        if (gained_confidence > full_confidence)
        {
            return full_confidence;
        }

        return static_cast<std::uint16_t>(gained_confidence);
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

    std::uint32_t absolute_time_difference_ms(std::uint32_t left_time_ms, std::uint32_t right_time_ms)
    {
        if (left_time_ms >= right_time_ms)
        {
            return left_time_ms - right_time_ms;
        }

        return right_time_ms - left_time_ms;
    }

    std::int64_t calculate_distance_um(std::int64_t delta_x_um, std::int64_t delta_y_um)
    {
        const double delta_x = static_cast<double>(delta_x_um);
        const double delta_y = static_cast<double>(delta_y_um);
        const double distance = std::sqrt((delta_x * delta_x) + (delta_y * delta_y));

        return static_cast<std::int64_t>(distance);
    }

    void rotate_local_delta(std::int64_t local_delta_x_um, std::int64_t local_delta_y_um, std::int32_t rotation_urad, std::int64_t &global_delta_x_um, std::int64_t &global_delta_y_um)
    {
        const double rotation_rad = static_cast<double>(rotation_urad) / 1000000.0;
        const double cos_rotation = std::cos(rotation_rad);
        const double sin_rotation = std::sin(rotation_rad);
        const double rotated_x_um = (static_cast<double>(local_delta_x_um) * cos_rotation) - (static_cast<double>(local_delta_y_um) * sin_rotation);
        const double rotated_y_um = (static_cast<double>(local_delta_x_um) * sin_rotation) + (static_cast<double>(local_delta_y_um) * cos_rotation);

        global_delta_x_um = static_cast<std::int64_t>(std::llround(rotated_x_um));
        global_delta_y_um = static_cast<std::int64_t>(std::llround(rotated_y_um));
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

    accepted_sample convert_sample(const global_position_api::global_position_sample &sample, const motion_mcu_incoming_state::local_position_state &local_position)
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

        if ((local_position.has_pose == true) && (local_position.branch_id <= 1U) && (local_position.pose_id != 0U))
        {
            converted.has_local_reference = true;
            converted.local_x_um = local_position.x_um;
            converted.local_y_um = local_position.y_um;
            converted.local_heading_urad = local_position.heading_urad;
            converted.pose_id = local_position.pose_id;
            converted.branch_id = local_position.branch_id;
        }

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
        std::int64_t confidence_values[history_size] = {};
        std::uint8_t confidence_count = 0U;

        for (std::uint8_t index = 0U; index <= last_index; index++)
        {
            if (history[index].valid == false)
            {
                continue;
            }

            confidence_values[confidence_count] = static_cast<std::int64_t>(history[index].confidence_position);
            confidence_count++;
        }

        if (confidence_count == 0U)
        {
            return 0U;
        }

        const std::int64_t median_confidence = median_value(confidence_values, confidence_count);

        return static_cast<std::uint16_t>(median_confidence);
    }

    std::uint16_t calculate_samples_position_confidence(const accepted_sample *samples, std::uint8_t sample_count)
    {
        static std::int64_t confidence_values[huber_pca_heading_max_sample_count] = {};
        std::uint8_t confidence_count = 0U;

        for (std::uint8_t index = 0U; index < sample_count; index++)
        {
            if (samples[index].valid == false)
            {
                continue;
            }

            confidence_values[confidence_count] = static_cast<std::int64_t>(samples[index].confidence_position);
            confidence_count++;
        }

        if (confidence_count == 0U)
        {
            return 0U;
        }

        const std::int64_t median_confidence = median_value(confidence_values, confidence_count);

        return static_cast<std::uint16_t>(median_confidence);
    }

    std::uint16_t calculate_position_anchor_samples_confidence(const accepted_sample *samples, std::uint8_t sample_count)
    {
        static std::int64_t confidence_values[position_anchor_max_sample_count] = {};
        std::uint8_t confidence_count = 0U;

        for (std::uint8_t index = 0U; index < sample_count; index++)
        {
            if (samples[index].valid == false)
            {
                continue;
            }

            confidence_values[confidence_count] = static_cast<std::int64_t>(samples[index].confidence_position);
            confidence_count++;
        }

        if (confidence_count == 0U)
        {
            return 0U;
        }

        const std::int64_t median_confidence = median_value(confidence_values, confidence_count);

        return static_cast<std::uint16_t>(median_confidence);
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

            if (age_ms > chord_heading_max_window_age_ms)
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

        if (best_distance_um < chord_heading_min_distance_um)
        {
            return false;
        }

        oldest_index_out = best_index;
        distance_um_out = best_distance_um;

        return true;
    }

    void accept_heading_measurement(const heading_estimate_candidate &candidate)
    {
        if (stable.has_heading == false)
        {
            stable.has_heading = true;
            stable.heading_urad = candidate.heading_urad;
            stable.confidence_heading = candidate.confidence_heading;
            stable.heading_time_ms = candidate.heading_time_ms;
            stable.heading_sample_count = candidate.heading_sample_count;
            stable.heading_distance_um = candidate.heading_distance_um;
            stable.heading_reference_time_ms = candidate.heading_reference_time_ms;
            stable.heading_reference_sample_id = candidate.heading_reference_sample_id;
            stable.heading_estimated_delay_ms = candidate.heading_estimated_delay_ms;
            stable.heading_reference_pose_id = candidate.heading_reference_pose_id;
            stable.heading_reference_branch_id = candidate.heading_reference_branch_id;
            stable.heading_reference_x_um = candidate.heading_reference_x_um;
            stable.heading_reference_y_um = candidate.heading_reference_y_um;
            stable.heading_reference_z_um = candidate.heading_reference_z_um;
            stable.heading_fit_residual_um = candidate.heading_fit_residual_um;
            stable.candidate_anchor_position_confidence = candidate.candidate_anchor_position_confidence;
            stable.candidate_anchor_heading_confidence = candidate.candidate_anchor_heading_confidence;
            stable.candidate_anchor_adjusted_heading_confidence = candidate.candidate_anchor_adjusted_heading_confidence;
            stable.candidate_anchor_confidence = candidate.candidate_anchor_confidence;
            stable.huber_pca_used_sample_count = candidate.huber_pca_used_sample_count;
            stable.huber_pca_median_residual_um = candidate.huber_pca_median_residual_um;
            stable.huber_pca_max_residual_um = candidate.huber_pca_max_residual_um;
            stable.huber_pca_movement_distance_um = candidate.huber_pca_movement_distance_um;
            stable.huber_pca_window_age_ms = candidate.huber_pca_window_age_ms;
            stable.chord_used_sample_count = candidate.chord_used_sample_count;
            stable.chord_distance_um = candidate.chord_distance_um;
            stable.chord_max_line_error_um = candidate.chord_max_line_error_um;
            stable.chord_window_age_ms = candidate.chord_window_age_ms;
            return;
        }

        const std::int32_t difference_urad = normalize_angle_urad(candidate.heading_urad - stable.heading_urad);
        const std::int32_t step_urad = weighted_heading_step_urad(difference_urad, stable.confidence_heading, candidate.confidence_heading);

        stable.heading_urad = normalize_angle_urad(stable.heading_urad + step_urad);
        stable.confidence_heading = larger_confidence(stable.confidence_heading, candidate.confidence_heading);
        stable.heading_time_ms = candidate.heading_time_ms;
        stable.heading_sample_count = candidate.heading_sample_count;
        stable.heading_distance_um = candidate.heading_distance_um;
        stable.heading_reference_time_ms = candidate.heading_reference_time_ms;
        stable.heading_reference_sample_id = candidate.heading_reference_sample_id;
        stable.heading_estimated_delay_ms = candidate.heading_estimated_delay_ms;
        stable.heading_reference_pose_id = candidate.heading_reference_pose_id;
        stable.heading_reference_branch_id = candidate.heading_reference_branch_id;
        stable.heading_reference_x_um = candidate.heading_reference_x_um;
        stable.heading_reference_y_um = candidate.heading_reference_y_um;
        stable.heading_reference_z_um = candidate.heading_reference_z_um;
        stable.heading_fit_residual_um = candidate.heading_fit_residual_um;
        stable.candidate_anchor_position_confidence = candidate.candidate_anchor_position_confidence;
        stable.candidate_anchor_heading_confidence = candidate.candidate_anchor_heading_confidence;
        stable.candidate_anchor_adjusted_heading_confidence = candidate.candidate_anchor_adjusted_heading_confidence;
        stable.candidate_anchor_confidence = candidate.candidate_anchor_confidence;
        stable.huber_pca_used_sample_count = candidate.huber_pca_used_sample_count;
        stable.huber_pca_median_residual_um = candidate.huber_pca_median_residual_um;
        stable.huber_pca_max_residual_um = candidate.huber_pca_max_residual_um;
        stable.huber_pca_movement_distance_um = candidate.huber_pca_movement_distance_um;
        stable.huber_pca_window_age_ms = candidate.huber_pca_window_age_ms;
        stable.chord_used_sample_count = candidate.chord_used_sample_count;
        stable.chord_distance_um = candidate.chord_distance_um;
        stable.chord_max_line_error_um = candidate.chord_max_line_error_um;
        stable.chord_window_age_ms = candidate.chord_window_age_ms;
    }

    void update_heading_from_history_chord()
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
        const std::uint16_t distance_confidence = growth_to_confidence(heading_distance_um, chord_heading_min_distance_um, chord_heading_full_distance_um);
        const std::uint16_t count_confidence = sample_count_to_confidence(sample_count);
        const std::uint16_t line_confidence = range_to_confidence(max_line_error_um, chord_heading_good_line_error_um, chord_heading_zero_line_error_um);
        const std::uint16_t position_confidence = calculate_window_position_confidence(oldest_index);
        std::uint16_t measured_confidence = distance_confidence;

        measured_confidence = smaller_confidence(measured_confidence, count_confidence);
        measured_confidence = smaller_confidence(measured_confidence, line_confidence);
        measured_confidence = smaller_confidence(measured_confidence, position_confidence);

        if (measured_confidence < minimum_accepted_confidence)
        {
            return;
        }

        accepted_sample anchor_reference_sample = newest_sample;
        std::uint16_t anchor_position_confidence = 0U;
        std::uint32_t anchor_position_residual_um = 0U;
        static accepted_sample anchor_fit_samples[position_anchor_max_sample_count] = {};
        std::uint8_t anchor_fit_sample_count = 0U;

        for (std::uint8_t index = 0U; (index <= oldest_index) && (anchor_fit_sample_count < position_anchor_max_sample_count); index++)
        {
            anchor_fit_samples[anchor_fit_sample_count] = history[index];
            anchor_fit_sample_count++;
        }

        const bool has_anchor_reference = calculate_motion_compensated_anchor_reference(anchor_fit_samples, anchor_fit_sample_count, newest_sample, measured_heading_urad, anchor_reference_sample, anchor_position_confidence, anchor_position_residual_um);
        heading_estimate_candidate candidate = {};

        candidate.heading_urad = measured_heading_urad;
        candidate.confidence_heading = measured_confidence;
        candidate.heading_time_ms = newest_sample.received_time_ms;
        candidate.heading_sample_count = sample_count;
        candidate.heading_distance_um = heading_distance_um;
        candidate.heading_reference_time_ms = newest_sample.received_time_ms;
        candidate.heading_reference_sample_id = newest_sample.sample_id;
        candidate.heading_estimated_delay_ms = 0U;
        if (has_anchor_reference == true)
        {
            candidate.heading_reference_pose_id = newest_sample.pose_id;
            candidate.heading_reference_branch_id = newest_sample.branch_id;
        }

        candidate.heading_reference_x_um = anchor_reference_sample.x_um;
        candidate.heading_reference_y_um = anchor_reference_sample.y_um;
        candidate.heading_reference_z_um = anchor_reference_sample.z_um;
        candidate.heading_fit_residual_um = static_cast<std::uint32_t>(max_line_error_um);
        candidate.candidate_anchor_position_confidence = anchor_position_confidence;
        candidate.candidate_anchor_heading_confidence = measured_confidence;
        candidate.candidate_anchor_adjusted_heading_confidence = apply_confidence_gain(measured_confidence, candidate_anchor_heading_confidence_gain_permille);
        candidate.candidate_anchor_confidence = smaller_confidence(candidate.candidate_anchor_position_confidence, candidate.candidate_anchor_adjusted_heading_confidence);
        candidate.chord_used_sample_count = sample_count;
        candidate.chord_distance_um = heading_distance_um;
        candidate.chord_max_line_error_um = max_line_error_um;
        candidate.chord_window_age_ms = calculate_time_delta_ms(oldest_sample, newest_sample);
        accept_heading_measurement(candidate);
    }

    std::uint8_t collect_huber_pca_heading_samples(accepted_sample *samples_out)
    {
        if (history_count == 0U)
        {
            return 0U;
        }

        const accepted_sample newest_sample = history[0U];
        std::uint8_t sample_count = 0U;

        for (std::uint8_t index = 0U; index < history_count; index++)
        {
            const accepted_sample sample = history[index];

            if (sample.valid == false)
            {
                continue;
            }

            const std::uint32_t age_ms = calculate_time_delta_ms(sample, newest_sample);

            if (age_ms > huber_pca_heading_max_age_ms)
            {
                continue;
            }

            samples_out[sample_count] = sample;
            sample_count++;

            if (sample_count >= huber_pca_heading_max_sample_count)
            {
                return sample_count;
            }
        }

        return sample_count;
    }

    std::uint8_t find_huber_pca_reference_index(const accepted_sample *samples, std::uint8_t sample_count)
    {
        const accepted_sample newest_sample = samples[0U];
        const std::uint32_t target_time_ms = newest_sample.received_time_ms > huber_pca_heading_estimated_delay_ms ? newest_sample.received_time_ms - huber_pca_heading_estimated_delay_ms : 0U;
        std::uint8_t reference_index = 0U;
        std::uint32_t best_error_ms = UINT32_MAX;
        bool has_reference_sample = false;

        for (std::uint8_t index = 0U; index < sample_count; index++)
        {
            if (samples[index].has_local_reference == false)
            {
                continue;
            }

            const std::uint32_t error_ms = absolute_time_difference_ms(samples[index].received_time_ms, target_time_ms);

            if (error_ms < best_error_ms)
            {
                best_error_ms = error_ms;
                reference_index = index;
                has_reference_sample = true;
            }
        }

        if (has_reference_sample == false)
        {
            return 0U;
        }

        return reference_index;
    }

    std::uint8_t collect_huber_pca_reference_window_samples(const accepted_sample *samples, std::uint8_t sample_count, std::uint8_t reference_index, accepted_sample *samples_out)
    {
        const accepted_sample reference_sample = samples[reference_index];
        std::uint8_t output_count = 0U;

        for (std::uint8_t index = 0U; index < sample_count; index++)
        {
            const std::uint32_t time_error_ms = absolute_time_difference_ms(samples[index].received_time_ms, reference_sample.received_time_ms);

            if (time_error_ms > huber_pca_heading_reference_window_ms)
            {
                continue;
            }

            samples_out[output_count] = samples[index];
            output_count++;

            if (output_count >= huber_pca_heading_max_sample_count)
            {
                return output_count;
            }
        }

        return output_count;
    }

    bool update_huber_pca_iteration(const accepted_sample *samples, std::uint8_t sample_count, std::uint16_t *weights, std::uint32_t *residuals_um, double &direction_x, double &direction_y)
    {
        double weight_sum = 0.0;
        double center_x = 0.0;
        double center_y = 0.0;

        for (std::uint8_t index = 0U; index < sample_count; index++)
        {
            const double weight = static_cast<double>(weights[index]);
            center_x += weight * static_cast<double>(samples[index].x_um);
            center_y += weight * static_cast<double>(samples[index].y_um);
            weight_sum += weight;
        }

        if (weight_sum <= 0.0)
        {
            return false;
        }

        center_x /= weight_sum;
        center_y /= weight_sum;

        double xx = 0.0;
        double xy = 0.0;
        double yy = 0.0;

        for (std::uint8_t index = 0U; index < sample_count; index++)
        {
            const double weight = static_cast<double>(weights[index]);
            const double dx = static_cast<double>(samples[index].x_um) - center_x;
            const double dy = static_cast<double>(samples[index].y_um) - center_y;

            xx += weight * dx * dx;
            xy += weight * dx * dy;
            yy += weight * dy * dy;
        }

        xx /= weight_sum;
        xy /= weight_sum;
        yy /= weight_sum;

        if ((xx == 0.0) && (xy == 0.0) && (yy == 0.0))
        {
            return false;
        }

        const double line_angle_rad = 0.5 * std::atan2(2.0 * xy, xx - yy);

        direction_x = std::cos(line_angle_rad);
        direction_y = std::sin(line_angle_rad);

        for (std::uint8_t index = 0U; index < sample_count; index++)
        {
            const double dx = static_cast<double>(samples[index].x_um) - center_x;
            const double dy = static_cast<double>(samples[index].y_um) - center_y;
            const double residual_double = std::fabs((dx * direction_y) - (dy * direction_x));
            std::uint32_t residual_um = static_cast<std::uint32_t>(std::llround(residual_double));

            residuals_um[index] = residual_um;

            std::uint32_t robust_weight = full_confidence;

            if (residual_um > huber_pca_delta_um)
            {
                robust_weight = huber_pca_delta_um * static_cast<std::uint32_t>(full_confidence) / residual_um;

                if (robust_weight == 0U)
                {
                    robust_weight = 1U;
                }
            }

            std::uint32_t confidence = samples[index].confidence_position;

            if (confidence == 0U)
            {
                confidence = 1U;
            }

            std::uint32_t combined_weight = confidence * robust_weight / static_cast<std::uint32_t>(full_confidence);

            if (combined_weight == 0U)
            {
                combined_weight = 1U;
            }

            if (combined_weight > full_confidence)
            {
                combined_weight = full_confidence;
            }

            weights[index] = static_cast<std::uint16_t>(combined_weight);
        }

        return true;
    }

    std::uint32_t calculate_median_residual_um(const std::uint32_t *residuals_um, std::uint8_t sample_count)
    {
        static std::int64_t values[huber_pca_heading_max_sample_count] = {};

        for (std::uint8_t index = 0U; index < sample_count; index++)
        {
            values[index] = static_cast<std::int64_t>(residuals_um[index]);
        }

        return static_cast<std::uint32_t>(median_value(values, sample_count));
    }

    std::uint32_t calculate_max_residual_um(const std::uint32_t *residuals_um, std::uint8_t sample_count)
    {
        std::uint32_t max_residual_um = 0U;

        for (std::uint8_t index = 0U; index < sample_count; index++)
        {
            if (residuals_um[index] > max_residual_um)
            {
                max_residual_um = residuals_um[index];
            }
        }

        return max_residual_um;
    }

    void update_heading_from_history_huber_pca()
    {
        static accepted_sample samples[huber_pca_heading_max_sample_count] = {};
        const std::uint8_t collected_sample_count = collect_huber_pca_heading_samples(samples);

        if (collected_sample_count < huber_pca_heading_min_sample_count)
        {
            return;
        }

        const std::uint8_t reference_index = find_huber_pca_reference_index(samples, collected_sample_count);
        const accepted_sample reference_sample = samples[reference_index];
        static accepted_sample fit_samples[huber_pca_heading_max_sample_count] = {};
        const std::uint8_t fit_sample_count = collect_huber_pca_reference_window_samples(samples, collected_sample_count, reference_index, fit_samples);

        if (fit_sample_count < huber_pca_heading_min_sample_count)
        {
            return;
        }

        const accepted_sample newest_sample = fit_samples[0U];
        const accepted_sample oldest_sample = fit_samples[fit_sample_count - 1U];
        const std::int64_t movement_x_um = newest_sample.x_um - oldest_sample.x_um;
        const std::int64_t movement_y_um = newest_sample.y_um - oldest_sample.y_um;
        const std::int64_t movement_distance_um = calculate_distance_um(movement_x_um, movement_y_um);

        if (movement_distance_um < huber_pca_heading_min_distance_um)
        {
            return;
        }

        static std::uint16_t weights[huber_pca_heading_max_sample_count] = {};
        static std::uint32_t residuals_um[huber_pca_heading_max_sample_count] = {};
        double direction_x = 1.0;
        double direction_y = 0.0;

        for (std::uint8_t index = 0U; index < fit_sample_count; index++)
        {
            weights[index] = fit_samples[index].confidence_position;

            if (weights[index] == 0U)
            {
                weights[index] = 1U;
            }
        }

        for (std::uint8_t iteration = 0U; iteration < huber_pca_iteration_count; iteration++)
        {
            const bool fitted = update_huber_pca_iteration(fit_samples, fit_sample_count, weights, residuals_um, direction_x, direction_y);

            if (fitted == false)
            {
                return;
            }
        }

        const double dot = (direction_x * static_cast<double>(movement_x_um)) + (direction_y * static_cast<double>(movement_y_um));

        if (dot < 0.0)
        {
            direction_x = -direction_x;
            direction_y = -direction_y;
        }

        const double heading_rad = std::atan2(direction_y, direction_x);
        const std::int32_t measured_heading_urad = normalize_angle_urad(static_cast<std::int32_t>(std::llround(heading_rad * 1000000.0)));
        const std::uint32_t median_residual_um = calculate_median_residual_um(residuals_um, fit_sample_count);
        const std::uint32_t max_residual_um = calculate_max_residual_um(residuals_um, fit_sample_count);
        const std::uint16_t distance_confidence = growth_to_confidence(movement_distance_um, huber_pca_heading_min_distance_um, huber_pca_heading_full_distance_um);
        const std::uint16_t count_confidence = sample_count_to_confidence(fit_sample_count, huber_pca_heading_min_sample_count, huber_pca_heading_full_sample_count);
        const std::uint16_t residual_confidence = range_to_confidence(median_residual_um, huber_pca_good_residual_um, huber_pca_zero_residual_um);
        const std::uint16_t position_confidence = calculate_samples_position_confidence(fit_samples, fit_sample_count);
        std::uint16_t measured_confidence = distance_confidence;

        measured_confidence = smaller_confidence(measured_confidence, count_confidence);
        measured_confidence = smaller_confidence(measured_confidence, residual_confidence);
        measured_confidence = smaller_confidence(measured_confidence, position_confidence);

        if (measured_confidence < minimum_accepted_confidence)
        {
            return;
        }

        accepted_sample anchor_reference_sample = reference_sample;
        std::uint16_t anchor_position_confidence = 0U;
        std::uint32_t anchor_position_residual_um = 0U;
        const bool has_anchor_reference = calculate_motion_compensated_anchor_reference(fit_samples, fit_sample_count, reference_sample, measured_heading_urad, anchor_reference_sample, anchor_position_confidence, anchor_position_residual_um);
        heading_estimate_candidate candidate = {};

        candidate.heading_urad = measured_heading_urad;
        candidate.confidence_heading = measured_confidence;
        candidate.heading_time_ms = newest_sample.received_time_ms;
        candidate.heading_sample_count = fit_sample_count;
        candidate.heading_distance_um = movement_distance_um;
        candidate.heading_reference_time_ms = reference_sample.received_time_ms;
        candidate.heading_reference_sample_id = reference_sample.sample_id;
        candidate.heading_estimated_delay_ms = huber_pca_heading_estimated_delay_ms;
        if (has_anchor_reference == true)
        {
            candidate.heading_reference_pose_id = reference_sample.pose_id;
            candidate.heading_reference_branch_id = reference_sample.branch_id;
        }

        candidate.heading_reference_x_um = anchor_reference_sample.x_um;
        candidate.heading_reference_y_um = anchor_reference_sample.y_um;
        candidate.heading_reference_z_um = anchor_reference_sample.z_um;
        candidate.heading_fit_residual_um = median_residual_um;
        candidate.candidate_anchor_position_confidence = anchor_position_confidence;
        candidate.candidate_anchor_heading_confidence = measured_confidence;
        candidate.candidate_anchor_adjusted_heading_confidence = apply_confidence_gain(measured_confidence, candidate_anchor_heading_confidence_gain_permille);
        candidate.candidate_anchor_confidence = smaller_confidence(candidate.candidate_anchor_position_confidence, candidate.candidate_anchor_adjusted_heading_confidence);
        candidate.huber_pca_used_sample_count = fit_sample_count;
        candidate.huber_pca_median_residual_um = median_residual_um;
        candidate.huber_pca_max_residual_um = max_residual_um;
        candidate.huber_pca_movement_distance_um = static_cast<std::uint32_t>(movement_distance_um);
        candidate.huber_pca_window_age_ms = calculate_time_delta_ms(oldest_sample, newest_sample);
        accept_heading_measurement(candidate);
    }

    void update_heading_from_history()
    {
        if (global_heading_mode == filtered_global_position::filtered_global_heading_mode::chord)
        {
            update_heading_from_history_chord();
            return;
        }

        if (global_heading_mode == filtered_global_position::filtered_global_heading_mode::huber_pca)
        {
            update_heading_from_history_huber_pca();
            return;
        }
    }

    void clear_position_anchor_candidate()
    {
        stable.candidate_position_anchor_confidence = 0U;
        stable.position_anchor_sample_count = 0U;
        stable.position_anchor_median_residual_um = 0U;
        stable.position_anchor_window_age_ms = 0U;
    }

    std::uint8_t collect_position_anchor_samples(accepted_sample *samples_out)
    {
        if (history_count == 0U)
        {
            return 0U;
        }

        const accepted_sample newest_sample = history[0U];
        std::uint8_t sample_count = 0U;

        for (std::uint8_t index = 0U; index < history_count; index++)
        {
            const accepted_sample sample = history[index];

            if (sample.valid == false)
            {
                continue;
            }

            if (sample.has_local_reference == false)
            {
                continue;
            }

            const std::uint32_t age_ms = calculate_time_delta_ms(sample, newest_sample);

            if (age_ms > position_anchor_max_age_ms)
            {
                continue;
            }

            samples_out[sample_count] = sample;
            sample_count++;

            if (sample_count >= position_anchor_max_sample_count)
            {
                return sample_count;
            }
        }

        return sample_count;
    }

    bool find_position_anchor_reference_index(const accepted_sample *samples, std::uint8_t sample_count, std::uint8_t &reference_index_out)
    {
        const accepted_sample newest_sample = samples[0U];
        const std::uint32_t target_time_ms = newest_sample.received_time_ms > position_anchor_estimated_delay_ms ? newest_sample.received_time_ms - position_anchor_estimated_delay_ms : 0U;
        std::uint32_t best_error_ms = UINT32_MAX;
        bool has_reference_sample = false;

        for (std::uint8_t index = 0U; index < sample_count; index++)
        {
            if (samples[index].has_local_reference == false)
            {
                continue;
            }

            const std::uint32_t error_ms = absolute_time_difference_ms(samples[index].received_time_ms, target_time_ms);

            if (error_ms < best_error_ms)
            {
                best_error_ms = error_ms;
                reference_index_out = index;
                has_reference_sample = true;
            }
        }

        return has_reference_sample;
    }

    std::uint8_t collect_position_anchor_window_samples(const accepted_sample *samples, std::uint8_t sample_count, const accepted_sample &reference_sample, accepted_sample *samples_out)
    {
        std::uint8_t output_count = 0U;

        for (std::uint8_t index = 0U; index < sample_count; index++)
        {
            if (samples[index].has_local_reference == false)
            {
                continue;
            }

            if (samples[index].branch_id != reference_sample.branch_id)
            {
                continue;
            }

            const std::uint32_t time_error_ms = absolute_time_difference_ms(samples[index].received_time_ms, reference_sample.received_time_ms);

            if (time_error_ms > position_anchor_reference_window_ms)
            {
                continue;
            }

            samples_out[output_count] = samples[index];
            output_count++;

            if (output_count >= position_anchor_max_sample_count)
            {
                return output_count;
            }
        }

        return output_count;
    }

    bool calculate_weighted_center(const accepted_sample *samples, std::uint8_t sample_count, const std::uint16_t *weights, double &center_x, double &center_y, double &center_z)
    {
        double weight_sum = 0.0;

        center_x = 0.0;
        center_y = 0.0;
        center_z = 0.0;

        for (std::uint8_t index = 0U; index < sample_count; index++)
        {
            const double weight = static_cast<double>(weights[index]);

            center_x += weight * static_cast<double>(samples[index].x_um);
            center_y += weight * static_cast<double>(samples[index].y_um);
            center_z += weight * static_cast<double>(samples[index].z_um);
            weight_sum += weight;
        }

        if (weight_sum <= 0.0)
        {
            return false;
        }

        center_x /= weight_sum;
        center_y /= weight_sum;
        center_z /= weight_sum;

        return true;
    }

    bool update_position_anchor_huber_iteration(const accepted_sample *samples, std::uint8_t sample_count, std::uint16_t *weights, std::uint32_t *residuals_um, double &center_x, double &center_y, double &center_z)
    {
        if (calculate_weighted_center(samples, sample_count, weights, center_x, center_y, center_z) == false)
        {
            return false;
        }

        for (std::uint8_t index = 0U; index < sample_count; index++)
        {
            const double dx = static_cast<double>(samples[index].x_um) - center_x;
            const double dy = static_cast<double>(samples[index].y_um) - center_y;
            const double residual_double = std::sqrt((dx * dx) + (dy * dy));
            const std::uint32_t residual_um = static_cast<std::uint32_t>(std::llround(residual_double));

            residuals_um[index] = residual_um;

            std::uint32_t robust_weight = full_confidence;

            if (residual_um > position_anchor_huber_delta_um)
            {
                robust_weight = position_anchor_huber_delta_um * static_cast<std::uint32_t>(full_confidence) / residual_um;

                if (robust_weight == 0U)
                {
                    robust_weight = 1U;
                }
            }

            std::uint32_t confidence = samples[index].confidence_position;

            if (confidence == 0U)
            {
                confidence = 1U;
            }

            std::uint32_t combined_weight = confidence * robust_weight / static_cast<std::uint32_t>(full_confidence);

            if (combined_weight == 0U)
            {
                combined_weight = 1U;
            }

            if (combined_weight > full_confidence)
            {
                combined_weight = full_confidence;
            }

            weights[index] = static_cast<std::uint16_t>(combined_weight);
        }

        return true;
    }

    bool calculate_position_anchor_center(const accepted_sample *samples, std::uint8_t sample_count, accepted_sample &center_sample_out, std::uint32_t &median_residual_um_out)
    {
        static std::uint16_t weights[position_anchor_max_sample_count] = {};
        static std::uint32_t residuals_um[position_anchor_max_sample_count] = {};
        double center_x = 0.0;
        double center_y = 0.0;
        double center_z = 0.0;

        for (std::uint8_t index = 0U; index < sample_count; index++)
        {
            weights[index] = samples[index].confidence_position;

            if (weights[index] == 0U)
            {
                weights[index] = 1U;
            }
        }

        for (std::uint8_t iteration = 0U; iteration < position_anchor_iteration_count; iteration++)
        {
            if (update_position_anchor_huber_iteration(samples, sample_count, weights, residuals_um, center_x, center_y, center_z) == false)
            {
                return false;
            }
        }

        if (calculate_weighted_center(samples, sample_count, weights, center_x, center_y, center_z) == false)
        {
            return false;
        }

        for (std::uint8_t index = 0U; index < sample_count; index++)
        {
            const double dx = static_cast<double>(samples[index].x_um) - center_x;
            const double dy = static_cast<double>(samples[index].y_um) - center_y;
            const double residual_double = std::sqrt((dx * dx) + (dy * dy));

            residuals_um[index] = static_cast<std::uint32_t>(std::llround(residual_double));
        }

        center_sample_out.x_um = static_cast<std::int64_t>(std::llround(center_x));
        center_sample_out.y_um = static_cast<std::int64_t>(std::llround(center_y));
        center_sample_out.z_um = static_cast<std::int64_t>(std::llround(center_z));
        median_residual_um_out = calculate_median_residual_um(residuals_um, sample_count);

        return true;
    }

    void build_motion_compensated_position_anchor_samples(const accepted_sample *samples, std::uint8_t sample_count, const accepted_sample &reference_sample, std::int32_t reference_rotation_urad, accepted_sample *samples_out)
    {
        for (std::uint8_t index = 0U; index < sample_count; index++)
        {
            const std::int64_t local_delta_x_um = samples[index].local_x_um - reference_sample.local_x_um;
            const std::int64_t local_delta_y_um = samples[index].local_y_um - reference_sample.local_y_um;
            std::int64_t global_delta_x_um = 0;
            std::int64_t global_delta_y_um = 0;

            rotate_local_delta(local_delta_x_um, local_delta_y_um, reference_rotation_urad, global_delta_x_um, global_delta_y_um);

            samples_out[index] = samples[index];
            samples_out[index].x_um = samples[index].x_um - global_delta_x_um;
            samples_out[index].y_um = samples[index].y_um - global_delta_y_um;
        }
    }

    bool calculate_motion_compensated_anchor_reference(const accepted_sample *samples, std::uint8_t sample_count, const accepted_sample &reference_sample, std::int32_t measured_heading_urad, accepted_sample &center_sample_out, std::uint16_t &reference_position_confidence_out, std::uint32_t &median_residual_um_out)
    {
        reference_position_confidence_out = 0U;
        median_residual_um_out = 0U;

        if (reference_sample.has_local_reference == false)
        {
            return false;
        }

        const std::int32_t reference_rotation_urad = normalize_angle_urad(measured_heading_urad - reference_sample.local_heading_urad);
        static accepted_sample usable_samples[position_anchor_max_sample_count] = {};
        std::uint8_t usable_sample_count = 0U;

        for (std::uint8_t index = 0U; index < sample_count; index++)
        {
            if (samples[index].valid == false)
            {
                continue;
            }

            if (samples[index].has_local_reference == false)
            {
                continue;
            }

            if (samples[index].branch_id != reference_sample.branch_id)
            {
                continue;
            }

            usable_samples[usable_sample_count] = samples[index];
            usable_sample_count++;

            if (usable_sample_count >= position_anchor_max_sample_count)
            {
                break;
            }
        }

        if (usable_sample_count < position_anchor_min_sample_count)
        {
            return false;
        }

        static accepted_sample compensated_samples[position_anchor_max_sample_count] = {};
        build_motion_compensated_position_anchor_samples(usable_samples, usable_sample_count, reference_sample, reference_rotation_urad, compensated_samples);

        accepted_sample center_sample = reference_sample;

        if (calculate_position_anchor_center(compensated_samples, usable_sample_count, center_sample, median_residual_um_out) == false)
        {
            return false;
        }

        const std::uint16_t sample_count_confidence = sample_count_to_confidence(usable_sample_count, position_anchor_min_sample_count, position_anchor_full_sample_count);
        const std::uint16_t position_confidence = calculate_position_anchor_samples_confidence(compensated_samples, usable_sample_count);
        const std::uint16_t residual_confidence = range_to_confidence(median_residual_um_out, position_anchor_good_residual_um, position_anchor_zero_residual_um);
        std::uint16_t reference_confidence = sample_count_confidence;

        reference_confidence = smaller_confidence(reference_confidence, position_confidence);
        reference_confidence = smaller_confidence(reference_confidence, residual_confidence);

        center_sample_out = center_sample;
        reference_position_confidence_out = reference_confidence;

        return true;
    }

    void update_position_anchor_from_history(std::int32_t reference_rotation_urad)
    {
        static accepted_sample samples[position_anchor_max_sample_count] = {};
        const std::uint8_t collected_sample_count = collect_position_anchor_samples(samples);

        if (collected_sample_count < position_anchor_min_sample_count)
        {
            clear_position_anchor_candidate();
            return;
        }

        std::uint8_t reference_index = 0U;

        if (find_position_anchor_reference_index(samples, collected_sample_count, reference_index) == false)
        {
            clear_position_anchor_candidate();
            return;
        }

        const accepted_sample reference_sample = samples[reference_index];
        static accepted_sample fit_samples[position_anchor_max_sample_count] = {};
        const std::uint8_t fit_sample_count = collect_position_anchor_window_samples(samples, collected_sample_count, reference_sample, fit_samples);

        if (fit_sample_count < position_anchor_min_sample_count)
        {
            stable.candidate_position_anchor_confidence = 0U;
            stable.position_anchor_sample_count = fit_sample_count;
            stable.position_anchor_median_residual_um = 0U;
            stable.position_anchor_window_age_ms = 0U;
            return;
        }

        static accepted_sample compensated_samples[position_anchor_max_sample_count] = {};
        build_motion_compensated_position_anchor_samples(fit_samples, fit_sample_count, reference_sample, reference_rotation_urad, compensated_samples);

        accepted_sample center_sample = reference_sample;
        std::uint32_t median_residual_um = 0U;

        if (calculate_position_anchor_center(compensated_samples, fit_sample_count, center_sample, median_residual_um) == false)
        {
            stable.candidate_position_anchor_confidence = 0U;
            stable.position_anchor_sample_count = fit_sample_count;
            stable.position_anchor_median_residual_um = 0U;
            stable.position_anchor_window_age_ms = 0U;
            return;
        }

        const std::uint16_t sample_count_confidence = sample_count_to_confidence(fit_sample_count, position_anchor_min_sample_count, position_anchor_full_sample_count);
        const std::uint16_t position_confidence = calculate_position_anchor_samples_confidence(compensated_samples, fit_sample_count);
        const std::uint16_t residual_confidence = range_to_confidence(median_residual_um, position_anchor_good_residual_um, position_anchor_zero_residual_um);
        std::uint16_t anchor_confidence = sample_count_confidence;

        anchor_confidence = smaller_confidence(anchor_confidence, position_confidence);
        anchor_confidence = smaller_confidence(anchor_confidence, residual_confidence);

        stable.position_reference_time_ms = reference_sample.received_time_ms;
        stable.position_reference_sample_id = reference_sample.sample_id;
        stable.position_reference_pose_id = reference_sample.pose_id;
        stable.position_reference_branch_id = reference_sample.branch_id;
        stable.position_reference_x_um = center_sample.x_um;
        stable.position_reference_y_um = center_sample.y_um;
        stable.position_reference_z_um = center_sample.z_um;
        stable.position_reference_has_local_reference = reference_sample.has_local_reference;
        stable.position_reference_local_x_um = reference_sample.local_x_um;
        stable.position_reference_local_y_um = reference_sample.local_y_um;
        stable.position_reference_local_heading_urad = reference_sample.local_heading_urad;
        stable.candidate_position_anchor_confidence = anchor_confidence;
        stable.position_anchor_sample_count = fit_sample_count;
        stable.position_anchor_median_residual_um = median_residual_um;
        stable.position_anchor_window_age_ms = calculate_time_delta_ms(fit_samples[fit_sample_count - 1U], fit_samples[0U]);
    }

    void accept_initial_sample(const accepted_sample &sample)
    {
        stable.has_position = true;
        stable.x_um = sample.x_um;
        stable.y_um = sample.y_um;
        stable.z_um = sample.z_um;
        stable.confidence_position = sample.confidence_position;
        stable.position_time_ms = sample.received_time_ms;
        stable.position_reference_time_ms = sample.received_time_ms;
        stable.position_reference_sample_id = sample.sample_id;
        stable.position_reference_x_um = sample.x_um;
        stable.position_reference_y_um = sample.y_um;
        stable.position_reference_z_um = sample.z_um;
        stable.position_reference_has_local_reference = sample.has_local_reference;
        stable.position_reference_local_x_um = sample.local_x_um;
        stable.position_reference_local_y_um = sample.local_y_um;
        stable.position_reference_local_heading_urad = sample.local_heading_urad;

        if (sample.has_local_reference == true)
        {
            stable.position_reference_pose_id = sample.pose_id;
            stable.position_reference_branch_id = sample.branch_id;
        }

        push_history(sample);
        clear_position_anchor_candidate();
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
        stable.position_reference_time_ms = sample.received_time_ms;
        stable.position_reference_sample_id = sample.sample_id;
        stable.position_reference_x_um = stable.x_um;
        stable.position_reference_y_um = stable.y_um;
        stable.position_reference_z_um = stable.z_um;
        stable.position_reference_has_local_reference = sample.has_local_reference;
        stable.position_reference_local_x_um = sample.local_x_um;
        stable.position_reference_local_y_um = sample.local_y_um;
        stable.position_reference_local_heading_urad = sample.local_heading_urad;

        if (sample.has_local_reference == true)
        {
            stable.position_reference_pose_id = sample.pose_id;
            stable.position_reference_branch_id = sample.branch_id;
        }

        accepted_sample smoothed_sample = sample;
        smoothed_sample.x_um = stable.x_um;
        smoothed_sample.y_um = stable.y_um;
        smoothed_sample.z_um = stable.z_um;
        smoothed_sample.confidence_position = filtered_confidence;
        push_history(smoothed_sample);
        clear_position_anchor_candidate();
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
        output.heading_mode = static_cast<std::uint8_t>(global_heading_mode);
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
            output.position_reference_time_ms = stable.position_reference_time_ms;
            output.position_reference_sample_id = stable.position_reference_sample_id;
            output.position_reference_pose_id = stable.position_reference_pose_id;
            output.position_reference_branch_id = stable.position_reference_branch_id;
            output.position_reference_x_um = stable.position_reference_x_um;
            output.position_reference_y_um = stable.position_reference_y_um;
            output.position_reference_z_um = stable.position_reference_z_um;
            output.position_reference_has_local_reference = stable.position_reference_has_local_reference;
            output.position_reference_local_x_um = stable.position_reference_local_x_um;
            output.position_reference_local_y_um = stable.position_reference_local_y_um;
            output.position_reference_local_heading_urad = stable.position_reference_local_heading_urad;
            output.position_anchor_sample_count = stable.position_anchor_sample_count;
            output.position_anchor_median_residual_um = stable.position_anchor_median_residual_um;
            output.position_anchor_window_age_ms = stable.position_anchor_window_age_ms;
            output.candidate_position_anchor_confidence = apply_confidence_gain(multiply_confidence(stable.candidate_position_anchor_confidence, age_confidence), candidate_position_anchor_confidence_gain_permille);
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
            output.heading_reference_time_ms = stable.heading_reference_time_ms;
            output.heading_reference_sample_id = stable.heading_reference_sample_id;
            output.heading_estimated_delay_ms = stable.heading_estimated_delay_ms;
            output.heading_reference_pose_id = stable.heading_reference_pose_id;
            output.heading_reference_branch_id = stable.heading_reference_branch_id;
            output.heading_reference_x_um = stable.heading_reference_x_um;
            output.heading_reference_y_um = stable.heading_reference_y_um;
            output.heading_reference_z_um = stable.heading_reference_z_um;
            output.heading_fit_residual_um = stable.heading_fit_residual_um;
            output.candidate_anchor_position_confidence = stable.candidate_anchor_position_confidence;
            output.candidate_anchor_heading_confidence = stable.candidate_anchor_heading_confidence;
            output.candidate_anchor_adjusted_heading_confidence = stable.candidate_anchor_adjusted_heading_confidence;
            output.candidate_anchor_confidence = stable.candidate_anchor_confidence;
            output.huber_pca_used_sample_count = stable.huber_pca_used_sample_count;
            output.huber_pca_median_residual_um = stable.huber_pca_median_residual_um;
            output.huber_pca_max_residual_um = stable.huber_pca_max_residual_um;
            output.huber_pca_movement_distance_um = stable.huber_pca_movement_distance_um;
            output.huber_pca_window_age_ms = stable.huber_pca_window_age_ms;
            output.chord_used_sample_count = stable.chord_used_sample_count;
            output.chord_distance_um = stable.chord_distance_um;
            output.chord_max_line_error_um = stable.chord_max_line_error_um;
            output.chord_window_age_ms = stable.chord_window_age_ms;
        }

        latest_output = output;
        return latest_output;
    }

    filtered_global_position::output_snapshot rebuild_latest_output(std::uint32_t now_ms, bool preserve_sample_event_flags)
    {
        accepted_sample latest_sample = {};
        latest_sample.sample_id = latest_output.sample_id;
        latest_sample.request_id = latest_output.request_id;
        latest_sample.received_time_ms = latest_output.received_time_ms;
        latest_sample.confidence_position = latest_output.raw_confidence_position;

        const bool is_new_sample = preserve_sample_event_flags == true ? latest_output.is_new_sample : false;
        const bool accepted = preserve_sample_event_flags == true ? latest_output.accepted : false;
        const bool rejected = preserve_sample_event_flags == true ? latest_output.rejected : false;
        const std::uint16_t raw_confidence = latest_output.raw_confidence_position;
        const std::uint16_t history_confidence = latest_output.history_confidence;

        return build_output(now_ms, is_new_sample, accepted, rejected, latest_sample, raw_confidence, history_confidence);
    }

    filtered_global_position::output_snapshot reject_sample(std::uint32_t now_ms, const accepted_sample &sample, std::uint16_t raw_confidence, std::uint16_t history_confidence)
    {
        return build_output(now_ms, true, false, true, sample, raw_confidence, history_confidence);
    }

    filtered_global_position::output_snapshot process_new_sample(std::uint32_t now_ms, const global_position_api::global_position_sample &api_sample, const motion_mcu_incoming_state::local_position_state &local_position)
    {
        const accepted_sample sample = convert_sample(api_sample, local_position);
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
        candidate_anchor_heading_confidence_gain_permille = default_candidate_anchor_heading_confidence_gain_permille;
        candidate_position_anchor_confidence_gain_permille = default_candidate_position_anchor_confidence_gain_permille;
    }

    bool set_candidate_anchor_heading_confidence_gain_permille(std::uint16_t gain_permille)
    {
        if (gain_permille > maximum_candidate_anchor_confidence_gain_permille)
        {
            return false;
        }

        candidate_anchor_heading_confidence_gain_permille = gain_permille;
        return true;
    }

    std::uint16_t get_candidate_anchor_heading_confidence_gain_permille()
    {
        return candidate_anchor_heading_confidence_gain_permille;
    }

    bool set_candidate_position_anchor_confidence_gain_permille(std::uint16_t gain_permille)
    {
        if (gain_permille > maximum_candidate_anchor_confidence_gain_permille)
        {
            return false;
        }

        candidate_position_anchor_confidence_gain_permille = gain_permille;
        return true;
    }

    std::uint16_t get_candidate_position_anchor_confidence_gain_permille()
    {
        return candidate_position_anchor_confidence_gain_permille;
    }

    output_snapshot update(std::uint32_t now_ms, const motion_mcu_incoming_state::local_position_state &local_position)
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

        return process_new_sample(now_ms, api_sample, local_position);
    }

    output_snapshot update_position_anchor_reference(std::uint32_t now_ms, bool has_reference_rotation, std::int32_t reference_rotation_urad)
    {
        if (has_reference_rotation == false)
        {
            clear_position_anchor_candidate();
            return rebuild_latest_output(now_ms, true);
        }

        update_position_anchor_from_history(reference_rotation_urad);
        return rebuild_latest_output(now_ms, true);
    }

    output_snapshot read_output(std::uint32_t now_ms)
    {
        return rebuild_latest_output(now_ms, false);
    }
}
