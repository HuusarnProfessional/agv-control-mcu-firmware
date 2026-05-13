#include "../incoming_request_handler_declarations.hpp"

#include <cstddef>
#include <cstdio>

#include "../../../middleware_parse_helpers.hpp"
#include "../../../../../control/primitives/pause/pause_pipeline.hpp"
#include "../../handler_helpers.hpp"

namespace incoming_request_handlers
{
    bool handle_set_pause_ms()
    {
        std::uint32_t duration_ms = 0u;
        const bool parsed_ok = middleware_parse_helpers::read_uint32_and_end(duration_ms, 50000u);

        if (parsed_ok == false)
        {
            return false;
        }

        const bool pause_requested = pause_pipeline::request_pause(duration_ms);

        if (pause_requested == false)
        {
            return false;
        }

        char response[48] = {};
        const int formatted_length = std::snprintf(
            response,
            sizeof(response),
            "rsp:paused(%lu)",
            static_cast<unsigned long>(duration_ms));

        if ((formatted_length <= 0) || (static_cast<std::size_t>(formatted_length) >= sizeof(response)))
        {
            return false;
        }

        return handler_helpers::write_response_text(response);
    }
}
