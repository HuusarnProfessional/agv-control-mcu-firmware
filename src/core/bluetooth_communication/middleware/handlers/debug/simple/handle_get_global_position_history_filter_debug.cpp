#include "../debug_handler_declarations.hpp"

#include <cstddef>
#include <cstdint>

#include "../../../middleware_parse_helpers.hpp"
#include "../../handler_helpers.hpp"
#include "../../../../../position_sensorfusion/global_position_history_filter/global_position_history_filter.hpp"
#include "debug_handler_helpers.hpp"

namespace debug_handlers
{
    bool handle_get_global_position_history_filter_debug()
    {
        if (middleware_parse_helpers::read_end(debug_handler_helpers::timeout_us) == false)
        {
            return debug_handler_helpers::write_bad_format();
        }

        const global_position_history_filter::output_snapshot state = global_position_history_filter::read_output();

        char response[512] = {};
        std::size_t offset = 0U;

        const bool formatted =
            debug_handler_helpers::append_format(response, sizeof(response), offset, "global_position_history_filter_debug") &&
            debug_handler_helpers::append_format(response, sizeof(response), offset, " has_position %u", state.has_position ? 1U : 0U) &&
            debug_handler_helpers::append_format(response, sizeof(response), offset, " x_um %lld", static_cast<long long>(state.x_um)) &&
            debug_handler_helpers::append_format(response, sizeof(response), offset, " y_um %lld", static_cast<long long>(state.y_um)) &&
            debug_handler_helpers::append_format(response, sizeof(response), offset, " z_um %lld", static_cast<long long>(state.z_um)) &&
            debug_handler_helpers::append_format(response, sizeof(response), offset, " original_confidence_position %u", static_cast<unsigned>(state.original_confidence_position)) &&
            debug_handler_helpers::append_format(response, sizeof(response), offset, " history_confidence %u", static_cast<unsigned>(state.history_confidence)) &&
            debug_handler_helpers::append_format(response, sizeof(response), offset, " confidence_position %u", static_cast<unsigned>(state.confidence_position)) &&
            debug_handler_helpers::append_format(response, sizeof(response), offset, " has_heading %u", state.has_heading ? 1U : 0U) &&
            debug_handler_helpers::append_format(response, sizeof(response), offset, " heading_urad %ld", static_cast<long>(state.heading_urad)) &&
            debug_handler_helpers::append_format(response, sizeof(response), offset, " confidence_heading %u", static_cast<unsigned>(state.confidence_heading)) &&
            debug_handler_helpers::append_format(response, sizeof(response), offset, " is_new_sample %u", state.is_new_sample ? 1U : 0U) &&
            debug_handler_helpers::append_format(response, sizeof(response), offset, " rejected %u", state.rejected ? 1U : 0U) &&
            debug_handler_helpers::append_format(response, sizeof(response), offset, " sample_id %lu", static_cast<unsigned long>(state.sample_id)) &&
            debug_handler_helpers::append_format(response, sizeof(response), offset, " received_time_ms %lu", static_cast<unsigned long>(state.received_time_ms));

        if (formatted == false)
        {
            return false;
        }

        return handler_helpers::write_response_text(response);
    }
}
