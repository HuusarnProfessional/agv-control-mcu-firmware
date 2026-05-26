#include "../debug_handler_declarations.hpp"

#include "../../../middleware_parse_helpers.hpp"
#include "../../handler_helpers.hpp"
#include "../../../../../position_sensorfusion/position_sensorfusion_pipeline.hpp"
#include "debug_handler_helpers.hpp"

#include <cstdio>

namespace
{
    bool read_enabled(bool &enabled)
    {
        return middleware_parse_helpers::read_bool_and_end(enabled, debug_handler_helpers::timeout_us);
    }

    bool write_enabled_response(const char *name, bool enabled)
    {
        char response[64] = {};

        if (enabled == true)
        {
            (void)snprintf(response, sizeof(response), "%s enabled", name);
        }
        else
        {
            (void)snprintf(response, sizeof(response), "%s disabled", name);
        }

        return handler_helpers::write_response_text(response);
    }
}

namespace debug_handlers
{
    bool handle_set_position_sensorfusion_heading_anchor()
    {
        bool enabled = false;

        if (read_enabled(enabled) == false)
        {
            return debug_handler_helpers::write_bad_format();
        }

        position_sensorfusion_pipeline::set_heading_anchor_enabled(enabled);
        return write_enabled_response("position_sensorfusion_heading_anchor", enabled);
    }

    bool handle_set_position_sensorfusion_position_anchor()
    {
        bool enabled = false;

        if (read_enabled(enabled) == false)
        {
            return debug_handler_helpers::write_bad_format();
        }

        position_sensorfusion_pipeline::set_position_anchor_enabled(enabled);
        return write_enabled_response("position_sensorfusion_position_anchor", enabled);
    }

    bool handle_set_position_sensorfusion_position_anchor_direct_filtered_sample()
    {
        bool enabled = false;

        if (read_enabled(enabled) == false)
        {
            return debug_handler_helpers::write_bad_format();
        }

        position_sensorfusion_pipeline::set_position_anchor_direct_filtered_sample_mode(enabled);
        return write_enabled_response("position_anchor_direct_filtered_sample", enabled);
    }

    bool handle_set_position_sensorfusion_position_anchor_jump_guard()
    {
        bool enabled = false;

        if (read_enabled(enabled) == false)
        {
            return debug_handler_helpers::write_bad_format();
        }

        position_sensorfusion_pipeline::set_position_anchor_jump_guard_enabled(enabled);
        return write_enabled_response("position_anchor_jump_guard", enabled);
    }
}
