#include "../debug_handler_declarations.hpp"

#include <Arduino.h>

#include <cstddef>

#include "../../../middleware_parse_helpers.hpp"
#include "../../handler_helpers.hpp"
#include "../../../../../position_sensorfusion/position_sensorfusion_pipeline.hpp"
#include "debug_handler_helpers.hpp"

namespace
{
    const char *phase_to_text(std::uint8_t phase)
    {
        switch (phase)
        {
            case 0U: return "disabled";
            case 1U: return "monitoring";
            case 2U: return "holding_settle";
            case 3U: return "holding_collect";
            case 4U: return "request_wait_branch";
            case 5U: return "cooldown";
            case 6U: return "blocked";
            default: return "unknown";
        }
    }

    const char *cancel_reason_to_text(std::uint8_t reason)
    {
        switch (reason)
        {
            case 0U: return "none";
            case 1U: return "candidate_failed";
            case 2U: return "branch_timeout";
            case 3U: return "max_hold_timeout";
            default: return "unknown";
        }
    }
}

namespace debug_handlers
{
    bool handle_get_position_sensorfusion_stop_relocalize_status()
    {
        if (middleware_parse_helpers::read_end(debug_handler_helpers::timeout_us) == false)
        {
            return debug_handler_helpers::write_bad_format();
        }

        const auto status = position_sensorfusion_pipeline::read_stop_relocalize_status(millis());
        char response[560] = {};
        std::size_t offset = 0U;

        const bool formatted =
            debug_handler_helpers::append_format(response, sizeof(response), offset, "position_sensorfusion_stop_relocalize_status") &&
            debug_handler_helpers::append_format(response, sizeof(response), offset, " enabled %u", status.enabled ? 1U : 0U) &&
            debug_handler_helpers::append_format(response, sizeof(response), offset, " active %u", status.active ? 1U : 0U) &&
            debug_handler_helpers::append_format(response, sizeof(response), offset, " hold %u", status.external_hold ? 1U : 0U) &&
            debug_handler_helpers::append_format(response, sizeof(response), offset, " phase %s", phase_to_text(status.phase)) &&
            debug_handler_helpers::append_format(response, sizeof(response), offset, " local_conf %u", static_cast<unsigned>(status.local_confidence_position)) &&
            debug_handler_helpers::append_format(response, sizeof(response), offset, " filtered_conf %u", static_cast<unsigned>(status.filtered_confidence_position)) &&
            debug_handler_helpers::append_format(response, sizeof(response), offset, " filtered_ready %u", status.filtered_ready ? 1U : 0U) &&
            debug_handler_helpers::append_format(response, sizeof(response), offset, " filtered_age_ms %lu", static_cast<unsigned long>(status.filtered_age_ms)) &&
            debug_handler_helpers::append_format(response, sizeof(response), offset, " attempt_elapsed_ms %lu", static_cast<unsigned long>(status.attempt_elapsed_ms)) &&
            debug_handler_helpers::append_format(response, sizeof(response), offset, " phase_elapsed_ms %lu", static_cast<unsigned long>(status.phase_elapsed_ms)) &&
            debug_handler_helpers::append_format(response, sizeof(response), offset, " phase_required_ms %lu", static_cast<unsigned long>(status.phase_required_ms)) &&
            debug_handler_helpers::append_format(response, sizeof(response), offset, " cooldown_remaining_ms %lu", static_cast<unsigned long>(status.cooldown_remaining_ms)) &&
            debug_handler_helpers::append_format(response, sizeof(response), offset, " request_pending %u", status.request_pending ? 1U : 0U) &&
            debug_handler_helpers::append_format(response, sizeof(response), offset, " request_pose %u", static_cast<unsigned>(status.request_pose_id)) &&
            debug_handler_helpers::append_format(response, sizeof(response), offset, " request_branch %u", static_cast<unsigned>(status.request_branch_id)) &&
            debug_handler_helpers::append_format(response, sizeof(response), offset, " collected_samples %u", static_cast<unsigned>(status.collected_sample_count)) &&
            debug_handler_helpers::append_format(response, sizeof(response), offset, " last_cancel %s", cancel_reason_to_text(status.last_cancel_reason)) &&
            debug_handler_helpers::append_format(response, sizeof(response), offset, " last_activation %u", status.last_activation_performed ? 1U : 0U) &&
            debug_handler_helpers::append_format(response, sizeof(response), offset, " last_activation_conf %u", static_cast<unsigned>(status.last_activation_confidence)) &&
            debug_handler_helpers::append_format(response, sizeof(response), offset, " last_activation_time_ms %lu", static_cast<unsigned long>(status.last_activation_time_ms));

        if (formatted == false)
        {
            return false;
        }

        return handler_helpers::write_response_text(response);
    }
}
