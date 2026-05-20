#include "../debug_handler_declarations.hpp"

#include "../../../middleware_parse_helpers.hpp"
#include "../../handler_helpers.hpp"
#include "../../../../../position_sensorfusion/position_sensorfusion_pipeline.hpp"
#include "debug_handler_helpers.hpp"

namespace debug_handlers
{
    bool handle_set_position_sensorfusion_filtered_global_only()
    {
        bool enabled = false;

        if (middleware_parse_helpers::read_bool_and_end(enabled, debug_handler_helpers::timeout_us) == false)
        {
            return debug_handler_helpers::write_bad_format();
        }

        position_sensorfusion_pipeline::set_filtered_global_only_mode(enabled);

        if (enabled == true)
        {
            return handler_helpers::write_response_text("position_sensorfusion_filtered_global_only enabled");
        }

        return handler_helpers::write_response_text("position_sensorfusion_filtered_global_only disabled");
    }
}
