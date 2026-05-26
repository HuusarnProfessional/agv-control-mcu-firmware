#include "position_anchor.hpp"

#include "position_anchor_tuning.hpp"
#include "../internal/anchor_confidence.hpp"
#include "../../filtered_global/filtered_global.hpp"
#include "../../internal/confidence_math.hpp"
#include "../../internal/geometry_helpers.hpp"

#include <cmath>

namespace
{
    position_sensorfusion_anchors::candidate latest_candidate = {};
    bool direct_filtered_sample_mode = false;

    bool long_local_heading_gate_passes()
    {
        filtered_global::sample newest_sample = {};

        if (filtered_global::read_history_sample(0U, newest_sample) == false)
        {
            return false;
        }

        if (newest_sample.has_local_reference == false)
        {
            return false;
        }

        std::uint8_t collected_count = 0U;
        std::int32_t heading_samples_urad[position_anchor_tuning::long_heading_gate_sample_count] = {};

        for (std::uint8_t index = 0U; index < filtered_global::history_count(); index++)
        {
            filtered_global::sample sample = {};

            if (filtered_global::read_history_sample(index, sample) == false)
            {
                continue;
            }

            if (sample.has_local_reference == false)
            {
                continue;
            }

            if (sample.branch_id != newest_sample.branch_id)
            {
                continue;
            }

            heading_samples_urad[collected_count] = sample.local_heading_urad;
            collected_count++;

            if (collected_count >= position_anchor_tuning::long_heading_gate_sample_count)
            {
                break;
            }
        }

        if (collected_count < position_anchor_tuning::long_heading_gate_sample_count)
        {
            return false;
        }

        const std::int32_t newest_heading_urad = heading_samples_urad[0U];
        std::uint32_t maximum_span_urad = 0U;

        for (std::uint8_t index = 1U; index < collected_count; index++)
        {
            const std::uint32_t span_urad =
                position_sensorfusion_internal::absolute_angle_delta_urad(heading_samples_urad[index], newest_heading_urad);

            if (span_urad > maximum_span_urad)
            {
                maximum_span_urad = span_urad;
            }
        }

        return maximum_span_urad <= position_anchor_tuning::long_heading_gate_maximum_span_urad;
    }

    std::uint8_t collect_samples(filtered_global::sample *samples_out)
    {
        filtered_global::sample newest_sample = {};

        if (filtered_global::read_history_sample(0U, newest_sample) == false)
        {
            return 0U;
        }

        if (newest_sample.has_local_reference == false)
        {
            return 0U;
        }

        std::uint8_t sample_count = 0U;

        for (std::uint8_t index = 0U; index < filtered_global::history_count(); index++)
        {
            filtered_global::sample sample = {};

            if (filtered_global::read_history_sample(index, sample) == false)
            {
                continue;
            }

            if (sample.has_local_reference == false)
            {
                continue;
            }

            if (sample.branch_id != newest_sample.branch_id)
            {
                continue;
            }

            const std::uint32_t age_ms = position_sensorfusion_internal::elapsed_ms(newest_sample.received_time_ms, sample.received_time_ms);

            if (age_ms > position_anchor_tuning::maximum_age_ms)
            {
                continue;
            }

            samples_out[sample_count] = sample;
            sample_count++;

            if (sample_count >= position_anchor_tuning::maximum_sample_count)
            {
                return sample_count;
            }
        }

        return sample_count;
    }

    std::uint8_t find_reference_index(const filtered_global::sample *samples, std::uint8_t sample_count)
    {
        const std::uint32_t newest_time_ms = samples[0U].received_time_ms;
        std::uint32_t target_time_ms = 0U;

        if (newest_time_ms > position_anchor_tuning::estimated_delay_ms)
        {
            target_time_ms = newest_time_ms - position_anchor_tuning::estimated_delay_ms;
        }

        std::uint8_t reference_index = 0U;
        std::uint32_t best_error_ms = 0U;
        bool has_reference = false;

        for (std::uint8_t index = 0U; index < sample_count; index++)
        {
            const std::uint32_t error_ms = position_sensorfusion_internal::absolute_time_difference_ms(samples[index].received_time_ms, target_time_ms);

            if ((has_reference == false) || (error_ms < best_error_ms))
            {
                best_error_ms = error_ms;
                reference_index = index;
                has_reference = true;
            }
        }

        return reference_index;
    }

    std::uint32_t calculate_line_residual_um(std::int64_t start_x_um,
                                             std::int64_t start_y_um,
                                             std::int64_t end_x_um,
                                             std::int64_t end_y_um,
                                             std::int64_t point_x_um,
                                             std::int64_t point_y_um)
    {
        const double line_dx = static_cast<double>(end_x_um - start_x_um);
        const double line_dy = static_cast<double>(end_y_um - start_y_um);
        const double line_length = std::sqrt((line_dx * line_dx) + (line_dy * line_dy));

        if (line_length <= 1.0)
        {
            return 0U;
        }

        const double point_dx = static_cast<double>(point_x_um - start_x_um);
        const double point_dy = static_cast<double>(point_y_um - start_y_um);
        const double cross = std::abs((point_dx * line_dy) - (point_dy * line_dx));
        const double residual = cross / line_length;

        return static_cast<std::uint32_t>(std::llround(residual));
    }

    bool local_trajectory_is_straight(const filtered_global::sample *samples, std::uint8_t sample_count)
    {
        if (sample_count < 2U)
        {
            return false;
        }

        const filtered_global::sample &newest_sample = samples[0U];
        const filtered_global::sample &oldest_sample = samples[sample_count - 1U];
        const std::int64_t local_delta_x_um = newest_sample.local_x_um - oldest_sample.local_x_um;
        const std::int64_t local_delta_y_um = newest_sample.local_y_um - oldest_sample.local_y_um;
        const std::int64_t local_distance_um = position_sensorfusion_internal::calculate_distance_um(local_delta_x_um, local_delta_y_um);

        if (local_distance_um < position_anchor_tuning::minimum_trajectory_distance_um)
        {
            return false;
        }

        const std::uint32_t local_heading_span_urad =
            position_sensorfusion_internal::absolute_angle_delta_urad(newest_sample.local_heading_urad, oldest_sample.local_heading_urad);

        if (local_heading_span_urad > position_anchor_tuning::maximum_local_heading_span_urad)
        {
            return false;
        }

        for (std::uint8_t index = 1U; index + 1U < sample_count; index++)
        {
            const std::uint32_t local_residual_um =
                calculate_line_residual_um(oldest_sample.local_x_um,
                                           oldest_sample.local_y_um,
                                           newest_sample.local_x_um,
                                           newest_sample.local_y_um,
                                           samples[index].local_x_um,
                                           samples[index].local_y_um);

            if (local_residual_um > position_anchor_tuning::maximum_local_line_residual_um)
            {
                return false;
            }
        }

        return true;
    }

    void build_translation_samples(const filtered_global::sample *samples, std::uint8_t sample_count, std::int32_t rotation_urad, filtered_global::sample *translation_samples_out)
    {
        for (std::uint8_t index = 0U; index < sample_count; index++)
        {
            std::int64_t rotated_local_x_um = 0;
            std::int64_t rotated_local_y_um = 0;
            position_sensorfusion_internal::rotate_xy_um(samples[index].local_x_um, samples[index].local_y_um, rotation_urad, rotated_local_x_um, rotated_local_y_um);

            translation_samples_out[index] = samples[index];
            translation_samples_out[index].x_um = samples[index].x_um - rotated_local_x_um;
            translation_samples_out[index].y_um = samples[index].y_um - rotated_local_y_um;
        }
    }

    void rebuild_reference_sample(const filtered_global::sample &reference_sample, const filtered_global::sample &translation_center_sample, std::int32_t rotation_urad, filtered_global::sample &reference_position_sample_out)
    {
        std::int64_t rotated_local_x_um = 0;
        std::int64_t rotated_local_y_um = 0;
        position_sensorfusion_internal::rotate_xy_um(reference_sample.local_x_um, reference_sample.local_y_um, rotation_urad, rotated_local_x_um, rotated_local_y_um);

        reference_position_sample_out = reference_sample;
        reference_position_sample_out.x_um = translation_center_sample.x_um + rotated_local_x_um;
        reference_position_sample_out.y_um = translation_center_sample.y_um + rotated_local_y_um;
        reference_position_sample_out.z_um = translation_center_sample.z_um;
    }

    bool calculate_weighted_center(const filtered_global::sample *samples, std::uint8_t sample_count, const std::uint16_t *weights, double &center_x_out, double &center_y_out, double &center_z_out)
    {
        double weighted_x = 0.0;
        double weighted_y = 0.0;
        double weighted_z = 0.0;
        double weight_sum = 0.0;

        for (std::uint8_t index = 0U; index < sample_count; index++)
        {
            const double weight = static_cast<double>(weights[index]);
            weighted_x += weight * static_cast<double>(samples[index].x_um);
            weighted_y += weight * static_cast<double>(samples[index].y_um);
            weighted_z += weight * static_cast<double>(samples[index].z_um);
            weight_sum += weight;
        }

        if (weight_sum <= 0.0)
        {
            return false;
        }

        center_x_out = weighted_x / weight_sum;
        center_y_out = weighted_y / weight_sum;
        center_z_out = weighted_z / weight_sum;
        return true;
    }

    bool run_huber_center_iteration(const filtered_global::sample *samples, std::uint8_t sample_count, std::uint16_t *weights, std::uint32_t *residuals_um, double &center_x_out, double &center_y_out, double &center_z_out)
    {
        if (calculate_weighted_center(samples, sample_count, weights, center_x_out, center_y_out, center_z_out) == false)
        {
            return false;
        }

        for (std::uint8_t index = 0U; index < sample_count; index++)
        {
            const double delta_x = static_cast<double>(samples[index].x_um) - center_x_out;
            const double delta_y = static_cast<double>(samples[index].y_um) - center_y_out;
            const double residual = std::sqrt((delta_x * delta_x) + (delta_y * delta_y));
            residuals_um[index] = static_cast<std::uint32_t>(std::llround(residual));

            std::uint32_t huber_weight = position_sensorfusion_internal::full_confidence;

            if (residuals_um[index] > position_anchor_tuning::huber_delta_um)
            {
                huber_weight = position_anchor_tuning::huber_delta_um * static_cast<std::uint32_t>(position_sensorfusion_internal::full_confidence) / residuals_um[index];
            }

            if (huber_weight == 0U)
            {
                huber_weight = 1U;
            }

            std::uint32_t position_weight = samples[index].confidence_position;

            if (position_weight == 0U)
            {
                position_weight = 1U;
            }

            std::uint32_t combined_weight = position_weight * huber_weight / static_cast<std::uint32_t>(position_sensorfusion_internal::full_confidence);

            if (combined_weight == 0U)
            {
                combined_weight = 1U;
            }

            if (combined_weight > position_sensorfusion_internal::full_confidence)
            {
                combined_weight = position_sensorfusion_internal::full_confidence;
            }

            weights[index] = static_cast<std::uint16_t>(combined_weight);
        }

        return true;
    }

    bool calculate_huber_center(const filtered_global::sample *samples, std::uint8_t sample_count, filtered_global::sample &center_sample_out, std::uint32_t &median_residual_um_out)
    {
        std::uint16_t weights[position_anchor_tuning::maximum_sample_count] = {};
        std::uint32_t residuals_um[position_anchor_tuning::maximum_sample_count] = {};
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

        for (std::uint8_t iteration = 0U; iteration < position_anchor_tuning::iteration_count; iteration++)
        {
            if (run_huber_center_iteration(samples, sample_count, weights, residuals_um, center_x, center_y, center_z) == false)
            {
                return false;
            }
        }

        if (calculate_weighted_center(samples, sample_count, weights, center_x, center_y, center_z) == false)
        {
            return false;
        }

        center_sample_out.x_um = static_cast<std::int64_t>(std::llround(center_x));
        center_sample_out.y_um = static_cast<std::int64_t>(std::llround(center_y));
        center_sample_out.z_um = static_cast<std::int64_t>(std::llround(center_z));
        median_residual_um_out = anchor_confidence::median_residual_um(residuals_um, sample_count);

        return true;
    }

    position_sensorfusion_anchors::candidate build_candidate(const filtered_global::sample *samples, std::uint8_t sample_count, std::int32_t reference_rotation_urad)
    {
        position_sensorfusion_anchors::candidate candidate = {};
        const std::uint8_t reference_index = find_reference_index(samples, sample_count);
        const filtered_global::sample reference_sample = samples[reference_index];
        filtered_global::sample translation_samples[position_anchor_tuning::maximum_sample_count] = {};
        build_translation_samples(samples, sample_count, reference_rotation_urad, translation_samples);

        filtered_global::sample translation_center_sample = reference_sample;
        std::uint32_t median_residual_um = 0U;

        if (calculate_huber_center(translation_samples, sample_count, translation_center_sample, median_residual_um) == false)
        {
            return candidate;
        }

        filtered_global::sample center_sample = reference_sample;
        rebuild_reference_sample(reference_sample, translation_center_sample, reference_rotation_urad, center_sample);

        const std::uint16_t sample_count_confidence = position_sensorfusion_internal::sample_count_to_confidence(sample_count, position_anchor_tuning::minimum_sample_count, position_anchor_tuning::full_sample_count);
        const std::uint16_t position_confidence = anchor_confidence::median_position_confidence(samples, sample_count);
        const std::uint16_t residual_confidence = position_sensorfusion_internal::range_to_confidence(median_residual_um, position_anchor_tuning::good_residual_um, position_anchor_tuning::zero_residual_um);
        std::uint16_t confidence = position_sensorfusion_internal::smaller_confidence(sample_count_confidence, position_confidence);
        confidence = position_sensorfusion_internal::smaller_confidence(confidence, residual_confidence);
        confidence = position_sensorfusion_internal::apply_confidence_gain(confidence, position_anchor_tuning::confidence_gain_permille);

        candidate.valid = true;
        candidate.type = position_sensorfusion_anchors::anchor_type::position_only;
        candidate.confidence = confidence;
        candidate.reference.valid = true;
        candidate.reference.x_um = center_sample.x_um;
        candidate.reference.y_um = center_sample.y_um;
        candidate.reference.z_um = center_sample.z_um;
        candidate.reference.confidence_position = confidence;
        candidate.reference.sample_id = reference_sample.sample_id;
        candidate.reference.received_time_ms = reference_sample.received_time_ms;
        candidate.reference.has_local_reference = reference_sample.has_local_reference;
        candidate.reference.local_x_um = reference_sample.local_x_um;
        candidate.reference.local_y_um = reference_sample.local_y_um;
        candidate.reference.local_heading_urad = reference_sample.local_heading_urad;
        candidate.reference.pose_id = reference_sample.pose_id;
        candidate.reference.branch_id = reference_sample.branch_id;

        return candidate;
    }

    position_sensorfusion_anchors::candidate build_direct_filtered_candidate(const filtered_global::sample *samples, std::uint8_t sample_count)
    {
        position_sensorfusion_anchors::candidate candidate = {};

        if (sample_count == 0U)
        {
            return candidate;
        }

        const std::uint8_t reference_index = find_reference_index(samples, sample_count);
        const filtered_global::sample reference_sample = samples[reference_index];
        std::uint16_t confidence = reference_sample.confidence_position;

        if (confidence == 0U)
        {
            confidence = 1U;
        }

        confidence = position_sensorfusion_internal::apply_confidence_gain(confidence, position_anchor_tuning::confidence_gain_permille);

        candidate.valid = true;
        candidate.type = position_sensorfusion_anchors::anchor_type::position_only;
        candidate.confidence = confidence;
        candidate.reference.valid = true;
        candidate.reference.x_um = reference_sample.x_um;
        candidate.reference.y_um = reference_sample.y_um;
        candidate.reference.z_um = reference_sample.z_um;
        candidate.reference.confidence_position = confidence;
        candidate.reference.sample_id = reference_sample.sample_id;
        candidate.reference.received_time_ms = reference_sample.received_time_ms;
        candidate.reference.has_local_reference = reference_sample.has_local_reference;
        candidate.reference.local_x_um = reference_sample.local_x_um;
        candidate.reference.local_y_um = reference_sample.local_y_um;
        candidate.reference.local_heading_urad = reference_sample.local_heading_urad;
        candidate.reference.pose_id = reference_sample.pose_id;
        candidate.reference.branch_id = reference_sample.branch_id;

        return candidate;
    }
}

namespace position_anchor
{
    void init()
    {
        latest_candidate = {};
    }

    void set_direct_filtered_sample_mode(bool enabled)
    {
        direct_filtered_sample_mode = enabled;
        latest_candidate = {};
    }

    bool is_direct_filtered_sample_mode_enabled()
    {
        return direct_filtered_sample_mode;
    }

    position_sensorfusion_anchors::candidate update(bool has_reference_rotation, std::int32_t reference_rotation_urad)
    {
        if (has_reference_rotation == false)
        {
            latest_candidate = {};
            return latest_candidate;
        }

        filtered_global::sample samples[position_anchor_tuning::maximum_sample_count] = {};
        const std::uint8_t sample_count = collect_samples(samples);

        if (sample_count < position_anchor_tuning::minimum_sample_count)
        {
            latest_candidate = {};
            return latest_candidate;
        }

        if (long_local_heading_gate_passes() == false)
        {
            latest_candidate = {};
            return latest_candidate;
        }

        if (local_trajectory_is_straight(samples, sample_count) == false)
        {
            latest_candidate = {};
            return latest_candidate;
        }

        if (direct_filtered_sample_mode == true)
        {
            latest_candidate = build_direct_filtered_candidate(samples, sample_count);
            return latest_candidate;
        }

        latest_candidate = build_candidate(samples, sample_count, reference_rotation_urad);
        return latest_candidate;
    }
}
