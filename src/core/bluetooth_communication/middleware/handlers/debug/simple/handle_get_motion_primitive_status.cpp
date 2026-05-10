#include "../debug_handler_declarations.hpp"

#include <cstddef>

#include "../../../middleware_parse_helpers.hpp"
#include "../../handler_helpers.hpp"
#include "../../../../../control/primitives/motion_primitive/motion_primitive_status_monitor.hpp"
#include "debug_handler_helpers.hpp"

namespace debug_handlers
{
    bool handle_get_motion_primitive_status()
    {
        if (middleware_parse_helpers::read_end(debug_handler_helpers::timeout_us) == false)
        {
            return debug_handler_helpers::write_bad_format();
        }

        const motion_primitive_status_monitor::snapshot state = motion_primitive_status_monitor::read_snapshot();

        char response[512] = {};
        std::size_t offset = 0U;

        const bool formatted =
            debug_handler_helpers::append_format(response, sizeof(response), offset, "motion_primitive_status") &&
            debug_handler_helpers::append_format(response, sizeof(response), offset, " waiting %u", state.waiting ? 1U : 0U) &&
            debug_handler_helpers::append_format(response, sizeof(response), offset, " running %u", state.running ? 1U : 0U) &&
            debug_handler_helpers::append_format(response, sizeof(response), offset, " complete %u", state.complete ? 1U : 0U) &&
            debug_handler_helpers::append_format(response, sizeof(response), offset, " success %u", state.success ? 1U : 0U) &&
            debug_handler_helpers::append_format(response, sizeof(response), offset, " failed %u", state.failed ? 1U : 0U) &&
            debug_handler_helpers::append_format(response, sizeof(response), offset, " timed_out %u", state.timed_out ? 1U : 0U) &&
            debug_handler_helpers::append_format(response, sizeof(response), offset, " expected_primitive_id %u", static_cast<unsigned>(state.expected_primitive)) &&
            debug_handler_helpers::append_format(response, sizeof(response), offset, " send_time_ms %lu", static_cast<unsigned long>(state.send_time_ms)) &&
            debug_handler_helpers::append_format(response, sizeof(response), offset, " baseline_command_id %lu", static_cast<unsigned long>(state.baseline_command_id)) &&
            debug_handler_helpers::append_format(response, sizeof(response), offset, " tracked_command_id %lu", static_cast<unsigned long>(state.tracked_command_id)) &&
            debug_handler_helpers::append_format(response, sizeof(response), offset, " stm_valid %u", state.stm.valid ? 1U : 0U) &&
            debug_handler_helpers::append_format(response, sizeof(response), offset, " stm_command_id %lu", static_cast<unsigned long>(state.stm.command_id)) &&
            debug_handler_helpers::append_format(response, sizeof(response), offset, " stm_primitive_id %u", static_cast<unsigned>(state.stm.active_primitive_id)) &&
            debug_handler_helpers::append_format(response, sizeof(response), offset, " stm_state %u", static_cast<unsigned>(state.stm.state)) &&
            debug_handler_helpers::append_format(response, sizeof(response), offset, " stm_failure_code %u", static_cast<unsigned>(state.stm.failure_code)) &&
            debug_handler_helpers::append_format(response, sizeof(response), offset, " stm_status_time_ms %lu", static_cast<unsigned long>(state.stm.status_time_ms)) &&
            debug_handler_helpers::append_format(response, sizeof(response), offset, " stm_received_time_ms %lu", static_cast<unsigned long>(state.stm.received_time_ms));

        if (formatted == false)
        {
            return false;
        }

        return handler_helpers::write_response_text(response);
    }
}
