#include "../debug_handler_declarations.hpp"

#include <cstddef>

#include "../../../middleware_parse_helpers.hpp"
#include "../../handler_helpers.hpp"
#include "../../../../../motion_mcu_communication/motion_mcu_routes.hpp"
#include "../../../../../motion_mcu_communication/state/debug/debug_state.hpp"
#include "debug_handler_helpers.hpp"

namespace debug_handlers
{
    bool handle_get_motion_debug()
    {
        if (middleware_parse_helpers::read_end(debug_handler_helpers::timeout_us) == false)
        {
            return debug_handler_helpers::write_bad_format();
        }

        const std::uint8_t payload_id = static_cast<std::uint8_t>(motion_mcu_routes::incoming_payload_id::motion_debug);

        if (motion_mcu_debug_state::is_stream_enabled(payload_id) == false)
        {
            return debug_handler_helpers::write_stream_not_active("motion_debug");
        }

        const motion_mcu_debug_state::motion_debug_state state = motion_mcu_debug_state::get_motion_debug();

        if (state.valid == false)
        {
            return debug_handler_helpers::write_missing_data("motion_debug");
        }

        char response[768] = {};
        std::size_t offset = 0U;

        const bool formatted =
            debug_handler_helpers::append_format(response, sizeof(response), offset, "motion_debug") &&
            debug_handler_helpers::append_format(response, sizeof(response), offset, " drive_enabled %u", state.drive_enabled ? 1U : 0U) &&
            debug_handler_helpers::append_format(response, sizeof(response), offset, " motion_session_active %u", state.motion_session_active ? 1U : 0U) &&
            debug_handler_helpers::append_format(response, sizeof(response), offset, " safe_guard_latched %u", state.safe_guard_latched ? 1U : 0U) &&
            debug_handler_helpers::append_format(response, sizeof(response), offset, " motion_command_stale %u", state.motion_command_stale ? 1U : 0U) &&
            debug_handler_helpers::append_format(response, sizeof(response), offset, " has_pose %u", state.has_pose ? 1U : 0U) &&
            debug_handler_helpers::append_format(response, sizeof(response), offset, " pose_is_fresh %u", state.pose_is_fresh ? 1U : 0U) &&
            debug_handler_helpers::append_format(response, sizeof(response), offset, " heading_feedback_active %u", state.heading_feedback_active ? 1U : 0U) &&
            debug_handler_helpers::append_format(response, sizeof(response), offset, " has_not_ready_feedback %u", state.has_not_ready_feedback ? 1U : 0U) &&
            debug_handler_helpers::append_format(response, sizeof(response), offset, " has_invalid_feedback %u", state.has_invalid_feedback ? 1U : 0U) &&
            debug_handler_helpers::append_format(response, sizeof(response), offset, " linear_mm_s %ld", static_cast<long>(state.commanded_linear_velocity_mm_s)) &&
            debug_handler_helpers::append_format(response, sizeof(response), offset, " yaw_rate_mdeg_s %ld", static_cast<long>(state.commanded_yaw_rate_mdeg_s)) &&
            debug_handler_helpers::append_format(response, sizeof(response), offset, " corrected_yaw_rate_mdeg_s %ld", static_cast<long>(state.corrected_yaw_rate_mdeg_s)) &&
            debug_handler_helpers::append_format(response, sizeof(response), offset, " measured_yaw_rate_mdeg_s %ld", static_cast<long>(state.measured_yaw_rate_mdeg_s)) &&
            debug_handler_helpers::append_format(response, sizeof(response), offset, " outer_correction_mdeg_s %ld", static_cast<long>(state.outer_correction_mdeg_s)) &&
            debug_handler_helpers::append_format(response, sizeof(response), offset, " pose_confidence_heading %u", static_cast<unsigned>(state.pose_confidence_heading)) &&
            debug_handler_helpers::append_format(response, sizeof(response), offset, " pose_age_ms %lu", static_cast<unsigned long>(state.pose_age_ms)) &&
            debug_handler_helpers::append_format(response, sizeof(response), offset, " left_target_mm_s %ld", static_cast<long>(state.wheel_targets_mm_s[0])) &&
            debug_handler_helpers::append_format(response, sizeof(response), offset, " right_target_mm_s %ld", static_cast<long>(state.wheel_targets_mm_s[1])) &&
            debug_handler_helpers::append_format(response, sizeof(response), offset, " fl_speed_mm_s %ld", static_cast<long>(state.wheel_speeds_mm_s[0])) &&
            debug_handler_helpers::append_format(response, sizeof(response), offset, " fr_speed_mm_s %ld", static_cast<long>(state.wheel_speeds_mm_s[1])) &&
            debug_handler_helpers::append_format(response, sizeof(response), offset, " rl_speed_mm_s %ld", static_cast<long>(state.wheel_speeds_mm_s[2])) &&
            debug_handler_helpers::append_format(response, sizeof(response), offset, " rr_speed_mm_s %ld", static_cast<long>(state.wheel_speeds_mm_s[3])) &&
            debug_handler_helpers::append_format(response, sizeof(response), offset, " fl_sample_id %lu", static_cast<unsigned long>(state.wheel_sample_ids[0])) &&
            debug_handler_helpers::append_format(response, sizeof(response), offset, " fr_sample_id %lu", static_cast<unsigned long>(state.wheel_sample_ids[1])) &&
            debug_handler_helpers::append_format(response, sizeof(response), offset, " rl_sample_id %lu", static_cast<unsigned long>(state.wheel_sample_ids[2])) &&
            debug_handler_helpers::append_format(response, sizeof(response), offset, " rr_sample_id %lu", static_cast<unsigned long>(state.wheel_sample_ids[3])) &&
            debug_handler_helpers::append_format(response, sizeof(response), offset, " fl_sample_age_ms %lu", static_cast<unsigned long>(state.wheel_sample_age_ms[0])) &&
            debug_handler_helpers::append_format(response, sizeof(response), offset, " fr_sample_age_ms %lu", static_cast<unsigned long>(state.wheel_sample_age_ms[1])) &&
            debug_handler_helpers::append_format(response, sizeof(response), offset, " rl_sample_age_ms %lu", static_cast<unsigned long>(state.wheel_sample_age_ms[2])) &&
            debug_handler_helpers::append_format(response, sizeof(response), offset, " rr_sample_age_ms %lu", static_cast<unsigned long>(state.wheel_sample_age_ms[3])) &&
            debug_handler_helpers::append_format(response, sizeof(response), offset, " fl_has_new_sample %u", state.wheel_has_new_sample[0] ? 1U : 0U) &&
            debug_handler_helpers::append_format(response, sizeof(response), offset, " fr_has_new_sample %u", state.wheel_has_new_sample[1] ? 1U : 0U) &&
            debug_handler_helpers::append_format(response, sizeof(response), offset, " rl_has_new_sample %u", state.wheel_has_new_sample[2] ? 1U : 0U) &&
            debug_handler_helpers::append_format(response, sizeof(response), offset, " rr_has_new_sample %u", state.wheel_has_new_sample[3] ? 1U : 0U) &&
            debug_handler_helpers::append_format(response, sizeof(response), offset, " fl_has_measured_speed %u", state.wheel_has_measured_speed[0] ? 1U : 0U) &&
            debug_handler_helpers::append_format(response, sizeof(response), offset, " fr_has_measured_speed %u", state.wheel_has_measured_speed[1] ? 1U : 0U) &&
            debug_handler_helpers::append_format(response, sizeof(response), offset, " rl_has_measured_speed %u", state.wheel_has_measured_speed[2] ? 1U : 0U) &&
            debug_handler_helpers::append_format(response, sizeof(response), offset, " rr_has_measured_speed %u", state.wheel_has_measured_speed[3] ? 1U : 0U) &&
            debug_handler_helpers::append_format(response, sizeof(response), offset, " fl_u %d", static_cast<int>(state.wheel_drive_u[0])) &&
            debug_handler_helpers::append_format(response, sizeof(response), offset, " fr_u %d", static_cast<int>(state.wheel_drive_u[1])) &&
            debug_handler_helpers::append_format(response, sizeof(response), offset, " rl_u %d", static_cast<int>(state.wheel_drive_u[2])) &&
            debug_handler_helpers::append_format(response, sizeof(response), offset, " rr_u %d", static_cast<int>(state.wheel_drive_u[3])) &&
            debug_handler_helpers::append_format(response, sizeof(response), offset, " time_ms %lu", static_cast<unsigned long>(state.time_ms));

        if (formatted == false)
        {
            return false;
        }

        return handler_helpers::write_response_text(response);
    }
}
