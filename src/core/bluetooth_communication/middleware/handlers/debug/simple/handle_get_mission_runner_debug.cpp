#include "../debug_handler_declarations.hpp"

#include <cstddef>

#include "../../../middleware_parse_helpers.hpp"
#include "../../handler_helpers.hpp"
#include "../../../../../mission/mission_runner.hpp"
#include "debug_handler_helpers.hpp"

namespace debug_handlers
{
    bool handle_get_mission_runner_debug()
    {
        if (middleware_parse_helpers::read_end(debug_handler_helpers::timeout_us) == false)
        {
            return debug_handler_helpers::write_bad_format();
        }

        const mission_runner::snapshot state = mission_runner::read_snapshot();

        char response[256] = {};
        std::size_t offset = 0U;

        const bool formatted = debug_handler_helpers::append_format(response, sizeof(response), offset, "mission_runner_debug") && debug_handler_helpers::append_format(response, sizeof(response), offset, " running %u", state.is_running ? 1U : 0U) && debug_handler_helpers::append_format(response, sizeof(response), offset, " start_pending %u", state.start_pending ? 1U : 0U) && debug_handler_helpers::append_format(response, sizeof(response), offset, " branch_changed %u", state.branch_changed ? 1U : 0U) && debug_handler_helpers::append_format(response, sizeof(response), offset, " started_this_tick %u", state.started_this_tick ? 1U : 0U) && debug_handler_helpers::append_format(response, sizeof(response), offset, " current_part %u", static_cast<unsigned>(state.current_part)) && debug_handler_helpers::append_format(response, sizeof(response), offset, " part_count %u", static_cast<unsigned>(state.part_count)) && debug_handler_helpers::append_format(response, sizeof(response), offset, " pending_branch_id %u", static_cast<unsigned>(state.pending_branch_id)) && debug_handler_helpers::append_format(response, sizeof(response), offset, " pending_start_time_ms %lu", static_cast<unsigned long>(state.pending_start_time_ms)) && debug_handler_helpers::append_format(response, sizeof(response), offset, " branch_changed_time_ms %lu", static_cast<unsigned long>(state.branch_changed_time_ms));

        if (formatted == false)
        {
            return false;
        }

        return handler_helpers::write_response_text(response);
    }
}
