#include "../debug_handler_declarations.hpp"

#include "../../../middleware_parse_helpers.hpp"
#include "../../handler_helpers.hpp"
#include "../../../../../position_sensorfusion/filtered_global_position/filtered_global_position.hpp"
#include "../../../../../position_sensorfusion/global_reference_selector/global_reference_selector.hpp"
#include "debug_handler_helpers.hpp"

namespace debug_handlers
{
    bool handle_set_position_sensorfusion_heading_anchor()
    {
        bool enabled = false;

        if (middleware_parse_helpers::read_bool_and_end(enabled, debug_handler_helpers::timeout_us) == false)
        {
            return debug_handler_helpers::write_bad_format();
        }

        global_reference_selector::set_heading_anchor_enabled(enabled);

        if (enabled == true)
        {
            return handler_helpers::write_response_text("position_sensorfusion_heading_anchor enabled");
        }

        return handler_helpers::write_response_text("position_sensorfusion_heading_anchor disabled");
    }

    bool handle_set_position_sensorfusion_position_anchor()
    {
        bool enabled = false;

        if (middleware_parse_helpers::read_bool_and_end(enabled, debug_handler_helpers::timeout_us) == false)
        {
            return debug_handler_helpers::write_bad_format();
        }

        global_reference_selector::set_position_anchor_enabled(enabled);

        if (enabled == true)
        {
            return handler_helpers::write_response_text("position_sensorfusion_position_anchor enabled");
        }

        return handler_helpers::write_response_text("position_sensorfusion_position_anchor disabled");
    }

    bool handle_set_filtered_global_position_anchor_trajectory_gate()
    {
        bool enabled = false;

        if (middleware_parse_helpers::read_bool_and_end(enabled, debug_handler_helpers::timeout_us) == false)
        {
            return debug_handler_helpers::write_bad_format();
        }

        filtered_global_position::set_position_anchor_trajectory_gate_enabled(enabled);

        if (enabled == true)
        {
            return handler_helpers::write_response_text("filtered_global_position_anchor_trajectory_gate enabled");
        }

        return handler_helpers::write_response_text("filtered_global_position_anchor_trajectory_gate disabled");
    }
}
