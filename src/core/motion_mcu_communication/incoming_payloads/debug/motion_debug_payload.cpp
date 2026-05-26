#include "motion_debug_payload.hpp"

#include <Arduino.h>

#include <cstddef>

#include "../../payload_helper_functions.hpp"
#include "../../state/debug/debug_state.hpp"

namespace motion_debug_payload
{
    void handle(const std::uint8_t *payload_data, std::uint8_t payload_length)
    {
        motion_mcu_debug_state::motion_debug_state state = motion_mcu_debug_state::get_motion_debug();
        state.valid = false;
        state.received_time_ms = millis();

        std::size_t offset = 0U;

        const bool has_drive_enabled = payload_helper_functions::read_bool(payload_data, payload_length, offset, state.drive_enabled);
        const bool has_motion_session_active = payload_helper_functions::read_bool(payload_data, payload_length, offset + 1U, state.motion_session_active);
        const bool has_safe_guard_latched = payload_helper_functions::read_bool(payload_data, payload_length, offset + 2U, state.safe_guard_latched);
        const bool has_motion_command_stale = payload_helper_functions::read_bool(payload_data, payload_length, offset + 3U, state.motion_command_stale);
        const bool has_has_pose = payload_helper_functions::read_bool(payload_data, payload_length, offset + 4U, state.has_pose);
        const bool has_pose_is_fresh = payload_helper_functions::read_bool(payload_data, payload_length, offset + 5U, state.pose_is_fresh);
        const bool has_heading_feedback_active = payload_helper_functions::read_bool(payload_data, payload_length, offset + 6U, state.heading_feedback_active);
        const bool has_not_ready_feedback = payload_helper_functions::read_bool(payload_data, payload_length, offset + 7U, state.has_not_ready_feedback);
        const bool has_invalid_feedback = payload_helper_functions::read_bool(payload_data, payload_length, offset + 8U, state.has_invalid_feedback);

        if ((has_drive_enabled == false) ||
            (has_motion_session_active == false) ||
            (has_safe_guard_latched == false) ||
            (has_motion_command_stale == false) ||
            (has_has_pose == false) ||
            (has_pose_is_fresh == false) ||
            (has_heading_feedback_active == false) ||
            (has_not_ready_feedback == false) ||
            (has_invalid_feedback == false))
        {
            return;
        }

        offset = 9U;

        const bool has_commanded_linear_velocity = payload_helper_functions::read_i32_le(payload_data, payload_length, offset, state.commanded_linear_velocity_mm_s);
        const bool has_commanded_yaw_rate = payload_helper_functions::read_i32_le(payload_data, payload_length, offset + 4U, state.commanded_yaw_rate_mdeg_s);
        const bool has_corrected_yaw_rate = payload_helper_functions::read_i32_le(payload_data, payload_length, offset + 8U, state.corrected_yaw_rate_mdeg_s);
        const bool has_measured_yaw_rate = payload_helper_functions::read_i32_le(payload_data, payload_length, offset + 12U, state.measured_yaw_rate_mdeg_s);
        const bool has_outer_correction = payload_helper_functions::read_i32_le(payload_data, payload_length, offset + 16U, state.outer_correction_mdeg_s);
        const bool has_pose_confidence_heading = payload_helper_functions::read_u16_le(payload_data, payload_length, offset + 20U, state.pose_confidence_heading);
        const bool has_pose_age_ms = payload_helper_functions::read_u32_le(payload_data, payload_length, offset + 22U, state.pose_age_ms);

        if ((has_commanded_linear_velocity == false) ||
            (has_commanded_yaw_rate == false) ||
            (has_corrected_yaw_rate == false) ||
            (has_measured_yaw_rate == false) ||
            (has_outer_correction == false) ||
            (has_pose_confidence_heading == false) ||
            (has_pose_age_ms == false))
        {
            return;
        }

        offset = 26U;

        for (std::uint8_t wheel = 0U; wheel < 4U; ++wheel)
        {
            if (payload_helper_functions::read_i32_le(payload_data, payload_length, offset, state.wheel_targets_mm_s[wheel]) == false)
            {
                return;
            }

            offset += 4U;
        }

        for (std::uint8_t wheel = 0U; wheel < 4U; ++wheel)
        {
            if (payload_helper_functions::read_i32_le(payload_data, payload_length, offset, state.wheel_speeds_mm_s[wheel]) == false)
            {
                return;
            }

            offset += 4U;
        }

        for (std::uint8_t wheel = 0U; wheel < 4U; ++wheel)
        {
            if (payload_helper_functions::read_u32_le(payload_data, payload_length, offset, state.wheel_sample_ids[wheel]) == false)
            {
                return;
            }

            offset += 4U;
        }

        for (std::uint8_t wheel = 0U; wheel < 4U; ++wheel)
        {
            if (payload_helper_functions::read_u32_le(payload_data, payload_length, offset, state.wheel_sample_age_ms[wheel]) == false)
            {
                return;
            }

            offset += 4U;
        }

        for (std::uint8_t wheel = 0U; wheel < 4U; ++wheel)
        {
            if (payload_helper_functions::read_bool(payload_data, payload_length, offset, state.wheel_has_new_sample[wheel]) == false)
            {
                return;
            }

            offset += 1U;
        }

        for (std::uint8_t wheel = 0U; wheel < 4U; ++wheel)
        {
            if (payload_helper_functions::read_bool(payload_data, payload_length, offset, state.wheel_has_measured_speed[wheel]) == false)
            {
                return;
            }

            offset += 1U;
        }

        for (std::uint8_t wheel = 0U; wheel < 4U; ++wheel)
        {
            if (payload_helper_functions::read_i16_le(payload_data, payload_length, offset, state.wheel_drive_u[wheel]) == false)
            {
                return;
            }

            offset += 2U;
        }

        if (payload_helper_functions::read_u32_le(payload_data, payload_length, offset, state.time_ms) == false)
        {
            return;
        }

        state.valid = true;
        motion_mcu_debug_state::set_motion_debug(state);
    }
}
