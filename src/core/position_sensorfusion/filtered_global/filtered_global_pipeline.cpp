#include "filtered_global_pipeline.hpp"

#include "filtered_global_tuning.hpp"
#include "../../global_positioning/global_position_api.hpp"
#include "../internal/confidence_math.hpp"
#include "../internal/geometry_helpers.hpp"

namespace
{
    struct local_history_entry
    {
        bool valid = false;
        motion_mcu_incoming_state::local_position_state pose = {};
    };

    local_history_entry local_history[filtered_global_tuning::local_history_size] = {};
    std::uint8_t local_history_count = 0U;
    std::uint32_t last_local_history_time_ms = 0U;
    bool has_global_sample_id = false;
    std::uint32_t last_global_sample_id = 0U;

    std::uint16_t quality_factor_to_confidence(std::uint8_t quality_factor)
    {
        const std::uint16_t confidence = static_cast<std::uint16_t>(quality_factor) * 10U;

        if (confidence > position_sensorfusion_internal::full_confidence)
        {
            return position_sensorfusion_internal::full_confidence;
        }

        return confidence;
    }

    void push_local_history(const motion_mcu_incoming_state::local_position_state &local_position)
    {
        if (local_position.has_pose == false)
        {
            return;
        }

        if (local_position.received_time_ms == 0U)
        {
            return;
        }

        if (local_position.received_time_ms == last_local_history_time_ms)
        {
            return;
        }

        const std::uint8_t last_index = filtered_global_tuning::local_history_size - 1U;

        for (std::uint8_t index = last_index; index > 0U; index--)
        {
            local_history[index] = local_history[index - 1U];
        }

        local_history[0U].valid = true;
        local_history[0U].pose = local_position;
        last_local_history_time_ms = local_position.received_time_ms;

        if (local_history_count < filtered_global_tuning::local_history_size)
        {
            local_history_count++;
        }
    }

    bool find_local_reference(std::uint32_t target_time_ms, motion_mcu_incoming_state::local_position_state &local_position_out)
    {
        std::uint32_t best_error_ms = 0U;
        bool has_position = false;

        for (std::uint8_t index = 0U; index < local_history_count; index++)
        {
            if (local_history[index].valid == false)
            {
                continue;
            }

            const std::uint32_t time_error_ms = position_sensorfusion_internal::absolute_time_difference_ms(local_history[index].pose.received_time_ms, target_time_ms);

            if ((has_position == false) || (time_error_ms < best_error_ms))
            {
                best_error_ms = time_error_ms;
                local_position_out = local_history[index].pose;
                has_position = true;
            }
        }

        if (has_position == false)
        {
            return false;
        }

        if (best_error_ms > filtered_global_tuning::local_history_max_match_error_ms)
        {
            return false;
        }

        return true;
    }

    bool raw_global_sample_is_new(const global_position_api::global_position_sample &global_sample)
    {
        if (global_sample.valid == false)
        {
            return false;
        }

        if (global_sample.status != global_position_api::global_position_status::ok)
        {
            return false;
        }

        if ((has_global_sample_id == true) && (global_sample.sample_id == last_global_sample_id))
        {
            return false;
        }

        has_global_sample_id = true;
        last_global_sample_id = global_sample.sample_id;
        return true;
    }

    filtered_global::sample convert_sample(const global_position_api::global_position_sample &global_sample)
    {
        filtered_global::sample sample = {};
        sample.valid = true;
        sample.sample_id = global_sample.sample_id;
        sample.request_id = global_sample.request_id;
        sample.received_time_ms = global_sample.received_time_ms;
        sample.x_um = static_cast<std::int64_t>(global_sample.x_mm) * 1000LL;
        sample.y_um = static_cast<std::int64_t>(global_sample.y_mm) * 1000LL;
        sample.z_um = static_cast<std::int64_t>(global_sample.z_mm) * 1000LL;
        sample.raw_confidence_position = quality_factor_to_confidence(global_sample.quality_factor);
        sample.confidence_position = sample.raw_confidence_position;

        motion_mcu_incoming_state::local_position_state local_reference = {};

        if (find_local_reference(global_sample.received_time_ms, local_reference) == false)
        {
            return sample;
        }

        if ((local_reference.pose_id == 0U) || (local_reference.branch_id > 1U))
        {
            return sample;
        }

        sample.has_local_reference = true;
        sample.local_x_um = local_reference.x_um;
        sample.local_y_um = local_reference.y_um;
        sample.local_heading_urad = local_reference.heading_urad;
        sample.pose_id = local_reference.pose_id;
        sample.branch_id = local_reference.branch_id;

        return sample;
    }
}

namespace filtered_global_pipeline
{
    void init()
    {
        for (std::uint8_t index = 0U; index < filtered_global_tuning::local_history_size; index++)
        {
            local_history[index] = {};
        }

        local_history_count = 0U;
        last_local_history_time_ms = 0U;
        has_global_sample_id = false;
        last_global_sample_id = 0U;
        filtered_global::init();
    }

    filtered_global::output_snapshot tick(std::uint32_t now_ms, const motion_mcu_incoming_state::local_position_state &local_position)
    {
        push_local_history(local_position);

        global_position_api::global_position_sample raw_sample = {};

        if (global_position_api::read_sample(raw_sample) == false)
        {
            return filtered_global::read_output(now_ms);
        }

        if (raw_global_sample_is_new(raw_sample) == false)
        {
            return filtered_global::read_output(now_ms);
        }

        return filtered_global::update(convert_sample(raw_sample), now_ms);
    }
}
