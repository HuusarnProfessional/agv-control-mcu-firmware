#include "../debug_handler_declarations.hpp"

#include "../../../middleware_parse_helpers.hpp"
#include "../../handler_helpers.hpp"
#include "../../../../../mission/mission_debug_overrides.hpp"
#include "debug_handler_helpers.hpp"

#include <cstdio>

namespace
{
    bool write_enabled_response(bool enabled)
    {
        char response[64] = {};

        if (enabled == true)
        {
            (void)snprintf(response, sizeof(response), "mission_end_rotate_dummy enabled");
        }
        else
        {
            (void)snprintf(response, sizeof(response), "mission_end_rotate_dummy disabled");
        }

        return handler_helpers::write_response_text(response);
    }
}

namespace debug_handlers
{
    bool handle_set_mission_end_rotate_dummy()
    {
        bool enabled = false;

        if (middleware_parse_helpers::read_bool_and_end(enabled, debug_handler_helpers::timeout_us) == false)
        {
            return debug_handler_helpers::write_bad_format();
        }

        mission_debug_overrides::set_replace_rotate_end_command_with_dummy_enabled(enabled);
        return write_enabled_response(enabled);
    }
}
