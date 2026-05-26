#include "position_trace_logger.hpp"

#include <cmath>
#include <cstdarg>
#include <cstdio>

#include "../global_positioning/global_position_api.hpp"
#include "../position_sensorfusion/internal/geometry_helpers.hpp"

namespace
{
    constexpr std::uint16_t default_period_ms = 200U;
    constexpr std::uint16_t minimum_period_ms = 50U;
    constexpr std::uint16_t trace_capacity = 256U;
    constexpr std::uint8_t packet_sample_limit = 3U;

    constexpr std::uint16_t flag_local_valid = 1U << 0U;
    constexpr std::uint16_t flag_filtered_valid = 1U << 1U;
    constexpr std::uint16_t flag_raw_valid = 1U << 2U;
    constexpr std::uint16_t flag_sensorfusion_valid = 1U << 3U;
    constexpr std::uint16_t flag_filtered_new = 1U << 4U;
    constexpr std::uint16_t flag_filtered_accepted = 1U << 5U;
    constexpr std::uint16_t flag_filtered_rejected = 1U << 6U;
    constexpr std::uint16_t flag_anchor_event = 1U << 7U;
    constexpr std::uint16_t flag_anchor_decision = 1U << 8U;

    struct trace_sample
    {
        std::uint32_t time_ms = 0U;
        std::uint16_t flags = 0U;
        std::int32_t local_x_mm = 0;
        std::int32_t local_y_mm = 0;
        std::int32_t local_heading_urad = 0;
        std::int32_t stitched_local_x_mm = 0;
        std::int32_t stitched_local_y_mm = 0;
        std::int32_t stitched_local_heading_urad = 0;
        std::uint16_t local_confidence_position = 0U;
        std::uint16_t local_confidence_heading = 0U;
        std::uint16_t local_pose_id = 0U;
        std::uint8_t local_branch_id = 0U;
        std::int32_t filtered_x_mm = 0;
        std::int32_t filtered_y_mm = 0;
        std::uint16_t filtered_confidence = 0U;
        std::uint16_t filtered_raw_confidence = 0U;
        std::uint16_t filtered_history_confidence = 0U;
        std::uint32_t filtered_sample_id = 0U;
        std::int32_t filtered_motion_heading_urad = 0;
        std::int32_t raw_x_mm = 0;
        std::int32_t raw_y_mm = 0;
        std::uint8_t raw_quality = 0U;
        std::uint32_t raw_sample_id = 0U;
        std::int32_t sensorfusion_x_mm = 0;
        std::int32_t sensorfusion_y_mm = 0;
        std::int32_t sensorfusion_heading_urad = 0;
        std::int32_t filtered_minus_sensorfusion_x_mm = 0;
        std::int32_t filtered_minus_sensorfusion_y_mm = 0;
        std::int32_t filtered_minus_sensorfusion_forward_mm = 0;
        std::int32_t filtered_minus_sensorfusion_lateral_mm = 0;
        std::int32_t transform_rotation_urad = 0;
        std::uint16_t sensorfusion_confidence_position = 0U;
        std::uint16_t sensorfusion_confidence_heading = 0U;
        std::uint16_t sensorfusion_pose_id = 0U;
        std::uint8_t sensorfusion_branch_id = 0U;
        std::uint8_t event_action = 0U;
        std::uint8_t event_anchor_type = 0U;
        std::uint16_t event_pose_id = 0U;
        std::uint8_t event_branch_id = 0U;
        std::uint16_t event_confidence = 0U;
        std::int32_t event_x_mm = 0;
        std::int32_t event_y_mm = 0;
        std::int32_t event_saved_global_heading_urad = 0;
        std::int32_t event_activation_local_heading_urad = 0;
        std::uint8_t decision_state = 0U;
        std::uint8_t decision_selected_type = 0U;
        std::uint8_t decision_reject_reason = 0U;
        std::uint16_t decision_required_confidence = 0U;
        std::uint16_t decision_local_confidence = 0U;
        std::uint16_t decision_heading_confidence = 0U;
        std::uint16_t decision_position_confidence = 0U;
        std::uint16_t decision_heading_pose_id = 0U;
        std::uint16_t decision_position_pose_id = 0U;
        std::uint32_t decision_heading_sample_id = 0U;
        std::uint32_t decision_position_sample_id = 0U;
        std::uint8_t decision_position_projection_valid = 0U;
        std::int32_t decision_position_reference_x_mm = 0;
        std::int32_t decision_position_reference_y_mm = 0;
        std::int32_t decision_position_projected_x_mm = 0;
        std::int32_t decision_position_projected_y_mm = 0;
        std::int32_t decision_position_jump_x_mm = 0;
        std::int32_t decision_position_jump_y_mm = 0;
        std::int32_t decision_position_projection_rotation_urad = 0;
    };

    struct stitched_local_state
    {
        bool valid = false;
        std::uint8_t branch_id = 0U;
        std::int64_t branch_origin_x_um = 0;
        std::int64_t branch_origin_y_um = 0;
        std::int32_t branch_origin_heading_urad = 0;
        std::int64_t current_x_um = 0;
        std::int64_t current_y_um = 0;
        std::int32_t current_heading_urad = 0;
    };

    struct filtered_motion_state
    {
        bool valid = false;
        std::int64_t x_um = 0;
        std::int64_t y_um = 0;
    };

    trace_sample trace_buffer[trace_capacity] = {};
    std::uint16_t oldest_index = 0U;
    std::uint16_t trace_count = 0U;
    std::uint32_t trace_dropped_count = 0U;
    std::uint16_t trace_period_ms = default_period_ms;
    std::uint32_t last_trace_time_ms = 0U;
    bool trace_enabled = true;
    stitched_local_state stitched_local = {};
    filtered_motion_state filtered_motion = {};

    std::int32_t um_to_mm(std::int64_t value_um)
    {
        return static_cast<std::int32_t>(value_um / 1000LL);
    }

    bool append_format(char *buffer, std::size_t capacity, std::size_t &offset, const char *format, ...)
    {
        if ((buffer == nullptr) || (format == nullptr) || (offset >= capacity))
        {
            return false;
        }

        va_list args;
        va_start(args, format);
        const int written = std::vsnprintf(buffer + offset, capacity - offset, format, args);
        va_end(args);

        if (written < 0)
        {
            return false;
        }

        const std::size_t written_size = static_cast<std::size_t>(written);

        if ((offset + written_size) >= capacity)
        {
            return false;
        }

        offset += written_size;
        return true;
    }

    std::uint32_t elapsed_ms(std::uint32_t now_ms, std::uint32_t previous_ms)
    {
        return now_ms - previous_ms;
    }

    bool should_capture(std::uint32_t now_ms, const position_trace_logger::anchor_event_snapshot &anchor_event)
    {
        if (anchor_event.valid == true)
        {
            return true;
        }

        if (last_trace_time_ms == 0U)
        {
            return true;
        }

        return elapsed_ms(now_ms, last_trace_time_ms) >= trace_period_ms;
    }

    void push_sample(const trace_sample &sample)
    {
        if (trace_count < trace_capacity)
        {
            const std::uint16_t write_index = static_cast<std::uint16_t>((oldest_index + trace_count) % trace_capacity);
            trace_buffer[write_index] = sample;
            trace_count++;
            return;
        }

        trace_buffer[oldest_index] = sample;
        oldest_index = static_cast<std::uint16_t>((oldest_index + 1U) % trace_capacity);
        trace_dropped_count++;
    }

    bool pop_sample(trace_sample &sample_out)
    {
        if (trace_count == 0U)
        {
            return false;
        }

        sample_out = trace_buffer[oldest_index];
        oldest_index = static_cast<std::uint16_t>((oldest_index + 1U) % trace_capacity);
        trace_count--;
        return true;
    }

    void update_stitched_local(const motion_mcu_incoming_state::local_position_state &local_position)
    {
        if (local_position.has_pose == false)
        {
            return;
        }

        if (stitched_local.valid == false)
        {
            stitched_local.valid = true;
            stitched_local.branch_id = local_position.branch_id;
            stitched_local.branch_origin_x_um = 0;
            stitched_local.branch_origin_y_um = 0;
            stitched_local.branch_origin_heading_urad = 0;
        }
        else if (local_position.branch_id != stitched_local.branch_id)
        {
            stitched_local.branch_origin_x_um = stitched_local.current_x_um;
            stitched_local.branch_origin_y_um = stitched_local.current_y_um;
            stitched_local.branch_origin_heading_urad = stitched_local.current_heading_urad;
            stitched_local.branch_id = local_position.branch_id;
        }

        std::int64_t rotated_local_x_um = 0;
        std::int64_t rotated_local_y_um = 0;
        position_sensorfusion_internal::rotate_xy_um(
            local_position.x_um,
            local_position.y_um,
            stitched_local.branch_origin_heading_urad,
            rotated_local_x_um,
            rotated_local_y_um);

        stitched_local.current_x_um = stitched_local.branch_origin_x_um + rotated_local_x_um;
        stitched_local.current_y_um = stitched_local.branch_origin_y_um + rotated_local_y_um;
        stitched_local.current_heading_urad = position_sensorfusion_internal::normalize_angle_urad(stitched_local.branch_origin_heading_urad + local_position.heading_urad);
    }

    std::int32_t calculate_heading_from_delta(std::int64_t delta_x_um, std::int64_t delta_y_um)
    {
        if ((delta_x_um == 0) && (delta_y_um == 0))
        {
            return 0;
        }

        const double heading_rad = std::atan2(static_cast<double>(delta_y_um), static_cast<double>(delta_x_um));
        return static_cast<std::int32_t>(std::llround(heading_rad * 1000000.0));
    }

    trace_sample build_sample(std::uint32_t now_ms,
                              const motion_mcu_incoming_state::local_position_state &local_position,
                              const filtered_global::output_snapshot &filtered_position,
                              const position_sensorfusion::output_snapshot &sensorfusion_position,
                              std::int32_t transform_rotation_urad,
                              const position_trace_logger::anchor_event_snapshot &anchor_event,
                              const position_trace_logger::anchor_decision_snapshot &anchor_decision)
    {
        trace_sample sample = {};
        sample.time_ms = now_ms;
        sample.transform_rotation_urad = transform_rotation_urad;

        update_stitched_local(local_position);

        if (local_position.has_pose == true)
        {
            sample.flags |= flag_local_valid;
            sample.local_x_mm = um_to_mm(local_position.x_um);
            sample.local_y_mm = um_to_mm(local_position.y_um);
            sample.local_heading_urad = local_position.heading_urad;
            sample.stitched_local_x_mm = um_to_mm(stitched_local.current_x_um);
            sample.stitched_local_y_mm = um_to_mm(stitched_local.current_y_um);
            sample.stitched_local_heading_urad = stitched_local.current_heading_urad;
            sample.local_confidence_position = local_position.confidence_position;
            sample.local_confidence_heading = local_position.confidence_heading;
            sample.local_pose_id = local_position.pose_id;
            sample.local_branch_id = local_position.branch_id;
        }

        if (filtered_position.has_position == true)
        {
            sample.flags |= flag_filtered_valid;
            sample.filtered_x_mm = um_to_mm(filtered_position.x_um);
            sample.filtered_y_mm = um_to_mm(filtered_position.y_um);
            sample.filtered_confidence = filtered_position.confidence_position;
            sample.filtered_raw_confidence = filtered_position.raw_confidence_position;
            sample.filtered_history_confidence = filtered_position.history_confidence;
            sample.filtered_sample_id = filtered_position.sample_id;

            if (filtered_motion.valid == true)
            {
                sample.filtered_motion_heading_urad = calculate_heading_from_delta(filtered_position.x_um - filtered_motion.x_um,
                                                                                  filtered_position.y_um - filtered_motion.y_um);
            }

            filtered_motion.valid = true;
            filtered_motion.x_um = filtered_position.x_um;
            filtered_motion.y_um = filtered_position.y_um;
        }

        if (filtered_position.is_new_sample == true)
        {
            sample.flags |= flag_filtered_new;
        }

        if (filtered_position.accepted == true)
        {
            sample.flags |= flag_filtered_accepted;
        }

        if (filtered_position.rejected == true)
        {
            sample.flags |= flag_filtered_rejected;
        }

        global_position_api::global_position_sample raw_sample = {};

        if ((global_position_api::read_sample(raw_sample) == true) && (raw_sample.valid == true) && (raw_sample.status == global_position_api::global_position_status::ok))
        {
            sample.flags |= flag_raw_valid;
            sample.raw_x_mm = raw_sample.x_mm;
            sample.raw_y_mm = raw_sample.y_mm;
            sample.raw_quality = raw_sample.quality_factor;
            sample.raw_sample_id = raw_sample.sample_id;
        }

        if (sensorfusion_position.has_pose == true)
        {
            sample.flags |= flag_sensorfusion_valid;
            sample.sensorfusion_x_mm = um_to_mm(sensorfusion_position.x_um);
            sample.sensorfusion_y_mm = um_to_mm(sensorfusion_position.y_um);
            sample.sensorfusion_heading_urad = sensorfusion_position.heading_urad;
            sample.sensorfusion_confidence_position = sensorfusion_position.confidence_position;
            sample.sensorfusion_confidence_heading = sensorfusion_position.confidence_heading;
            sample.sensorfusion_pose_id = sensorfusion_position.pose_id;
            sample.sensorfusion_branch_id = sensorfusion_position.branch_id;

            if (filtered_position.has_position == true)
            {
                const std::int64_t error_x_um = filtered_position.x_um - sensorfusion_position.x_um;
                const std::int64_t error_y_um = filtered_position.y_um - sensorfusion_position.y_um;
                sample.filtered_minus_sensorfusion_x_mm = um_to_mm(error_x_um);
                sample.filtered_minus_sensorfusion_y_mm = um_to_mm(error_y_um);

                std::int64_t forward_axis_x = 0;
                std::int64_t forward_axis_y = 0;
                position_sensorfusion_internal::rotate_xy_um(1000000, 0, sensorfusion_position.heading_urad, forward_axis_x, forward_axis_y);
                const double axis_x = static_cast<double>(forward_axis_x) / 1000000.0;
                const double axis_y = static_cast<double>(forward_axis_y) / 1000000.0;
                const double error_x = static_cast<double>(error_x_um);
                const double error_y = static_cast<double>(error_y_um);
                const double forward_um = (error_x * axis_x) + (error_y * axis_y);
                const double lateral_um = (-error_x * axis_y) + (error_y * axis_x);
                sample.filtered_minus_sensorfusion_forward_mm = static_cast<std::int32_t>(std::llround(forward_um / 1000.0));
                sample.filtered_minus_sensorfusion_lateral_mm = static_cast<std::int32_t>(std::llround(lateral_um / 1000.0));
            }
        }

        if (anchor_event.valid == true)
        {
            sample.flags |= flag_anchor_event;
            sample.event_action = static_cast<std::uint8_t>(anchor_event.action);
            sample.event_anchor_type = static_cast<std::uint8_t>(anchor_event.anchor_type);
            sample.event_pose_id = anchor_event.pose_id;
            sample.event_branch_id = anchor_event.branch_id;
            sample.event_confidence = anchor_event.confidence;
            sample.event_x_mm = um_to_mm(anchor_event.x_um);
            sample.event_y_mm = um_to_mm(anchor_event.y_um);
            sample.event_saved_global_heading_urad = anchor_event.saved_global_heading_urad;
            sample.event_activation_local_heading_urad = anchor_event.activation_local_heading_urad;
        }

        if (anchor_decision.valid == true)
        {
            sample.flags |= flag_anchor_decision;
            sample.decision_state = anchor_decision.selector_state;
            sample.decision_selected_type = anchor_decision.selected_type;
            sample.decision_reject_reason = anchor_decision.reject_reason;
            sample.decision_required_confidence = anchor_decision.required_confidence;
            sample.decision_local_confidence = anchor_decision.local_confidence;
            sample.decision_heading_confidence = anchor_decision.heading_confidence;
            sample.decision_position_confidence = anchor_decision.position_confidence;
            sample.decision_heading_pose_id = anchor_decision.heading_pose_id;
            sample.decision_position_pose_id = anchor_decision.position_pose_id;
            sample.decision_heading_sample_id = anchor_decision.heading_sample_id;
            sample.decision_position_sample_id = anchor_decision.position_sample_id;
            sample.decision_position_projection_valid = anchor_decision.position_projection_valid ? 1U : 0U;
            sample.decision_position_reference_x_mm = um_to_mm(anchor_decision.position_reference_x_um);
            sample.decision_position_reference_y_mm = um_to_mm(anchor_decision.position_reference_y_um);
            sample.decision_position_projected_x_mm = um_to_mm(anchor_decision.position_projected_x_um);
            sample.decision_position_projected_y_mm = um_to_mm(anchor_decision.position_projected_y_um);
            sample.decision_position_jump_x_mm = um_to_mm(anchor_decision.position_jump_x_um);
            sample.decision_position_jump_y_mm = um_to_mm(anchor_decision.position_jump_y_um);
            sample.decision_position_projection_rotation_urad = anchor_decision.position_projection_rotation_urad;
        }

        return sample;
    }

    bool append_sample(char *buffer, std::size_t capacity, std::size_t &offset, const trace_sample &sample)
    {
        return append_format(
            buffer,
            capacity,
            offset,
            "%lu,%u,%ld,%ld,%ld,%ld,%ld,%ld,%u,%u,%u,%u,%ld,%ld,%u,%u,%u,%lu,%ld,%ld,%u,%lu,%ld,%ld,%u,%lu,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%u,%u,%u,%u,%u,%u,%u,%u,%u,%ld,%ld,%ld,%ld,%u,%u,%u,%u,%u,%u,%u,%u,%u,%lu,%lu,%u,%ld,%ld,%ld,%ld,%ld,%ld,%ld",
            static_cast<unsigned long>(sample.time_ms),
            static_cast<unsigned>(sample.flags),
            static_cast<long>(sample.local_x_mm),
            static_cast<long>(sample.local_y_mm),
            static_cast<long>(sample.local_heading_urad),
            static_cast<long>(sample.stitched_local_x_mm),
            static_cast<long>(sample.stitched_local_y_mm),
            static_cast<long>(sample.stitched_local_heading_urad),
            static_cast<unsigned>(sample.local_confidence_position),
            static_cast<unsigned>(sample.local_confidence_heading),
            static_cast<unsigned>(sample.local_pose_id),
            static_cast<unsigned>(sample.local_branch_id),
            static_cast<long>(sample.filtered_x_mm),
            static_cast<long>(sample.filtered_y_mm),
            static_cast<unsigned>(sample.filtered_confidence),
            static_cast<unsigned>(sample.filtered_raw_confidence),
            static_cast<unsigned>(sample.filtered_history_confidence),
            static_cast<unsigned long>(sample.filtered_sample_id),
            static_cast<long>(sample.filtered_motion_heading_urad),
            static_cast<long>(sample.raw_x_mm),
            static_cast<long>(sample.raw_y_mm),
            static_cast<unsigned>(sample.raw_quality),
            static_cast<unsigned long>(sample.raw_sample_id),
            static_cast<long>(sample.sensorfusion_x_mm),
            static_cast<long>(sample.sensorfusion_y_mm),
            static_cast<long>(sample.sensorfusion_heading_urad),
            static_cast<long>(sample.filtered_minus_sensorfusion_x_mm),
            static_cast<long>(sample.filtered_minus_sensorfusion_y_mm),
            static_cast<long>(sample.filtered_minus_sensorfusion_forward_mm),
            static_cast<long>(sample.filtered_minus_sensorfusion_lateral_mm),
            static_cast<long>(sample.transform_rotation_urad),
            static_cast<unsigned>(sample.sensorfusion_confidence_position),
            static_cast<unsigned>(sample.sensorfusion_confidence_heading),
            static_cast<unsigned>(sample.sensorfusion_pose_id),
            static_cast<unsigned>(sample.sensorfusion_branch_id),
            static_cast<unsigned>(sample.event_action),
            static_cast<unsigned>(sample.event_anchor_type),
            static_cast<unsigned>(sample.event_pose_id),
            static_cast<unsigned>(sample.event_branch_id),
            static_cast<unsigned>(sample.event_confidence),
            static_cast<long>(sample.event_x_mm),
            static_cast<long>(sample.event_y_mm),
            static_cast<long>(sample.event_saved_global_heading_urad),
            static_cast<long>(sample.event_activation_local_heading_urad),
            static_cast<unsigned>(sample.decision_state),
            static_cast<unsigned>(sample.decision_selected_type),
            static_cast<unsigned>(sample.decision_reject_reason),
            static_cast<unsigned>(sample.decision_required_confidence),
            static_cast<unsigned>(sample.decision_local_confidence),
            static_cast<unsigned>(sample.decision_heading_confidence),
            static_cast<unsigned>(sample.decision_position_confidence),
            static_cast<unsigned>(sample.decision_heading_pose_id),
            static_cast<unsigned>(sample.decision_position_pose_id),
            static_cast<unsigned long>(sample.decision_heading_sample_id),
            static_cast<unsigned long>(sample.decision_position_sample_id),
            static_cast<unsigned>(sample.decision_position_projection_valid),
            static_cast<long>(sample.decision_position_reference_x_mm),
            static_cast<long>(sample.decision_position_reference_y_mm),
            static_cast<long>(sample.decision_position_projected_x_mm),
            static_cast<long>(sample.decision_position_projected_y_mm),
            static_cast<long>(sample.decision_position_jump_x_mm),
            static_cast<long>(sample.decision_position_jump_y_mm),
            static_cast<long>(sample.decision_position_projection_rotation_urad));
    }
}

namespace position_trace_logger
{
    void init()
    {
        clear();
        trace_period_ms = default_period_ms;
        trace_enabled = true;
    }

    void clear()
    {
        oldest_index = 0U;
        trace_count = 0U;
        trace_dropped_count = 0U;
        last_trace_time_ms = 0U;
        stitched_local = {};
        filtered_motion = {};
    }

    void set_enabled(bool enabled)
    {
        trace_enabled = enabled;
    }

    bool is_enabled()
    {
        return trace_enabled;
    }

    void set_period_ms(std::uint16_t period_ms)
    {
        if (period_ms < minimum_period_ms)
        {
            trace_period_ms = minimum_period_ms;
            return;
        }

        trace_period_ms = period_ms;
    }

    std::uint16_t get_period_ms()
    {
        return trace_period_ms;
    }

    std::uint16_t stored_count()
    {
        return trace_count;
    }

    std::uint32_t dropped_count()
    {
        return trace_dropped_count;
    }

    void tick(std::uint32_t now_ms,
              const motion_mcu_incoming_state::local_position_state &local_position,
              const filtered_global::output_snapshot &filtered_position,
              const position_sensorfusion::output_snapshot &sensorfusion_position,
              std::int32_t transform_rotation_urad,
              const anchor_event_snapshot &anchor_event,
              const anchor_decision_snapshot &anchor_decision)
    {
        if (trace_enabled == false)
        {
            return;
        }

        if (should_capture(now_ms, anchor_event) == false)
        {
            return;
        }

        push_sample(build_sample(now_ms, local_position, filtered_position, sensorfusion_position, transform_rotation_urad, anchor_event, anchor_decision));
        last_trace_time_ms = now_ms;
    }

    bool format_status(char *buffer_out, std::size_t capacity)
    {
        if ((buffer_out == nullptr) || (capacity == 0U))
        {
            return false;
        }

        const int length = std::snprintf(
            buffer_out,
            capacity,
            "position_trace_status enabled %u period_ms %u count %u dropped %lu capacity %u",
            trace_enabled ? 1U : 0U,
            static_cast<unsigned>(trace_period_ms),
            static_cast<unsigned>(trace_count),
            static_cast<unsigned long>(trace_dropped_count),
            static_cast<unsigned>(trace_capacity));

        return (length > 0) && (static_cast<std::size_t>(length) < capacity);
    }

    bool format_packet(char *buffer_out, std::size_t capacity)
    {
        if ((buffer_out == nullptr) || (capacity == 0U))
        {
            return false;
        }

        if (trace_count == 0U)
        {
            const int length = std::snprintf(buffer_out, capacity, "position_trace_empty");
            return (length > 0) && (static_cast<std::size_t>(length) < capacity);
        }

        trace_sample packet_samples[packet_sample_limit] = {};
        std::uint8_t packet_count = 0U;

        while ((packet_count < packet_sample_limit) && (pop_sample(packet_samples[packet_count]) == true))
        {
            packet_count++;
        }

        std::size_t offset = 0U;

        if (append_format(buffer_out, capacity, offset, "position_trace_packet %u ", static_cast<unsigned>(packet_count)) == false)
        {
            return false;
        }

        for (std::uint8_t index = 0U; index < packet_count; index++)
        {
            if (index > 0U)
            {
                if (append_format(buffer_out, capacity, offset, ";") == false)
                {
                    return false;
                }
            }

            if (append_sample(buffer_out, capacity, offset, packet_samples[index]) == false)
            {
                return false;
            }
        }

        return true;
    }
}
