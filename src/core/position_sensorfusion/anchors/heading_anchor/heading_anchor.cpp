#include "heading_anchor.hpp"

#include "heading_anchor_tuning.hpp"
#include "huber_pca_fit.hpp"
#include "../internal/anchor_confidence.hpp"
#include "../../filtered_global/filtered_global.hpp"
#include "../../internal/confidence_math.hpp"
#include "../../internal/geometry_helpers.hpp"

#include <cmath>

namespace
{
    position_sensorfusion_anchors::candidate latest_candidate = {};

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

        if (local_distance_um < heading_anchor_tuning::minimum_trajectory_distance_um)
        {
            return false;
        }

        const std::uint32_t local_heading_span_urad =
            position_sensorfusion_internal::absolute_angle_delta_urad(newest_sample.local_heading_urad, oldest_sample.local_heading_urad);

        if (local_heading_span_urad > heading_anchor_tuning::maximum_local_heading_span_urad)
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

            if (local_residual_um > heading_anchor_tuning::maximum_local_line_residual_um)
            {
                return false;
            }
        }

        return true;
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

            if (age_ms > heading_anchor_tuning::maximum_age_ms)
            {
                continue;
            }

            samples_out[sample_count] = sample;
            sample_count++;

            if (sample_count >= heading_anchor_tuning::maximum_sample_count)
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

        if (newest_time_ms > heading_anchor_tuning::estimated_delay_ms)
        {
            target_time_ms = newest_time_ms - heading_anchor_tuning::estimated_delay_ms;
        }

        std::uint8_t reference_index = 0U;
        std::uint32_t best_error_ms = 0U;
        bool has_reference = false;

        for (std::uint8_t index = 0U; index < sample_count; index++)
        {
            const std::uint32_t error_ms = position_sensorfusion_internal::absolute_time_difference_ms(samples[index].received_time_ms, target_time_ms);

            if ((has_reference == false) || (error_ms < best_error_ms))
            {
                reference_index = index;
                best_error_ms = error_ms;
                has_reference = true;
            }
        }

        return reference_index;
    }

    position_sensorfusion_anchors::candidate build_candidate(const filtered_global::sample *samples, std::uint8_t sample_count)
    {
        position_sensorfusion_anchors::candidate candidate = {};
        const filtered_global::sample newest_sample = samples[0U];
        const filtered_global::sample oldest_sample = samples[sample_count - 1U];
        const std::int64_t movement_x_um = newest_sample.x_um - oldest_sample.x_um;
        const std::int64_t movement_y_um = newest_sample.y_um - oldest_sample.y_um;
        const std::int64_t movement_distance_um = position_sensorfusion_internal::calculate_distance_um(movement_x_um, movement_y_um);

        if (movement_distance_um < heading_anchor_tuning::minimum_distance_um)
        {
            return candidate;
        }

        huber_pca_fit::fit_result fit_result = huber_pca_fit::fit(samples, sample_count);

        if (fit_result.valid == false)
        {
            return candidate;
        }

        const double movement_dot = (fit_result.direction_x * static_cast<double>(movement_x_um)) + (fit_result.direction_y * static_cast<double>(movement_y_um));

        if (movement_dot < 0.0)
        {
            fit_result.direction_x = -fit_result.direction_x;
            fit_result.direction_y = -fit_result.direction_y;
        }

        const double heading_rad = std::atan2(fit_result.direction_y, fit_result.direction_x);
        const std::int32_t heading_urad = position_sensorfusion_internal::normalize_angle_urad(static_cast<std::int32_t>(std::llround(heading_rad * 1000000.0)));
        const std::uint16_t distance_confidence = position_sensorfusion_internal::growth_to_confidence(movement_distance_um, heading_anchor_tuning::minimum_distance_um, heading_anchor_tuning::full_distance_um);
        const std::uint16_t sample_count_confidence = position_sensorfusion_internal::sample_count_to_confidence(sample_count, heading_anchor_tuning::minimum_sample_count, heading_anchor_tuning::full_sample_count);
        const std::uint16_t residual_confidence = position_sensorfusion_internal::range_to_confidence(fit_result.median_residual_um, heading_anchor_tuning::good_residual_um, heading_anchor_tuning::zero_residual_um);
        const std::uint16_t position_confidence = anchor_confidence::median_position_confidence(samples, sample_count);
        std::uint16_t heading_confidence = position_sensorfusion_internal::smaller_confidence(distance_confidence, sample_count_confidence);
        heading_confidence = position_sensorfusion_internal::smaller_confidence(heading_confidence, residual_confidence);
        heading_confidence = position_sensorfusion_internal::smaller_confidence(heading_confidence, position_confidence);
        heading_confidence = position_sensorfusion_internal::apply_confidence_gain(heading_confidence, heading_anchor_tuning::confidence_gain_permille);

        const std::uint8_t reference_index = find_reference_index(samples, sample_count);
        const filtered_global::sample reference_sample = samples[reference_index];

        candidate.valid = true;
        candidate.type = position_sensorfusion_anchors::anchor_type::heading_transform;
        candidate.confidence = heading_confidence;
        candidate.reference.valid = true;
        candidate.reference.has_heading = true;
        candidate.reference.x_um = reference_sample.x_um;
        candidate.reference.y_um = reference_sample.y_um;
        candidate.reference.z_um = reference_sample.z_um;
        candidate.reference.heading_urad = heading_urad;
        candidate.reference.confidence_position = position_confidence;
        candidate.reference.confidence_heading = heading_confidence;
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

namespace heading_anchor
{
    void init()
    {
        latest_candidate = {};
    }

    position_sensorfusion_anchors::candidate update()
    {
        filtered_global::sample samples[heading_anchor_tuning::maximum_sample_count] = {};
        const std::uint8_t sample_count = collect_samples(samples);

        if (sample_count < heading_anchor_tuning::minimum_sample_count)
        {
            latest_candidate = {};
            return latest_candidate;
        }

        if (local_trajectory_is_straight(samples, sample_count) == false)
        {
            latest_candidate = {};
            return latest_candidate;
        }

        latest_candidate = build_candidate(samples, sample_count);
        return latest_candidate;
    }
}
