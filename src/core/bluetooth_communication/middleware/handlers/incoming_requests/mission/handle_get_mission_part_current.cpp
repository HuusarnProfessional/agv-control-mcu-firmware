#include "../incoming_request_handler_declarations.hpp"

#include <cstdio>

#include "../../../middleware_parse_helpers.hpp"
#include "../../../../../mission/mission_runner.hpp"
#include "../../handler_helpers.hpp"

namespace
{
    constexpr std::uint32_t timeout_us = 50000u;
    constexpr std::size_t response_capacity = 48u;
}

namespace incoming_request_handlers
{
    bool handle_get_mission_part_current()
    {
        const bool parsed_ok = middleware_parse_helpers::read_end(timeout_us);

        if (parsed_ok == false)
        {
            return false;
        }

        std::uint16_t current_part = 0u;
        const bool has_current_part = mission_runner::get_current_part(current_part);

        if (has_current_part == false)
        {
            const bool fail_response_written = handler_helpers::write_response("rsp:fail(3)");

            if (fail_response_written == false)
            {
                return false;
            }

            return false;
        }

        char response[response_capacity] = {};
        const int formatted_length = std::snprintf(
            response,
            sizeof(response),
            "rsp:mission_part_current(%u)",
            static_cast<unsigned int>(current_part));

        if ((formatted_length <= 0) || (static_cast<std::size_t>(formatted_length) >= sizeof(response)))
        {
            return false;
        }

        const bool response_written = handler_helpers::write_response_text(response);

        if (response_written == false)
        {
            return false;
        }

        return true;
    }
}
