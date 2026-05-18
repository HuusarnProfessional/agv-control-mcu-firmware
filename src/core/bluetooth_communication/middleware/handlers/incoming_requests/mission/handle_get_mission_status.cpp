#include "../incoming_request_handler_declarations.hpp"

#include <cstdio>

#include "../../../middleware_parse_helpers.hpp"
#include "../../../../../mission/mission_runner.hpp"
#include "../../handler_helpers.hpp"

namespace
{
    constexpr std::uint32_t timeout_us = 50000u;
    constexpr std::size_t response_capacity = 64u;
}

namespace incoming_request_handlers
{
    bool handle_get_mission_status()
    {
        const bool parsed_ok = middleware_parse_helpers::read_end(timeout_us);

        if (parsed_ok == false)
        {
            return false;
        }

        const mission_runner::snapshot state = mission_runner::read_snapshot();

        char response[response_capacity] = {};
        const int formatted_length = std::snprintf(response, sizeof(response), "rsp:mission_status(%u,%u,%u)", state.is_running ? 1U : 0U, static_cast<unsigned int>(state.current_part), static_cast<unsigned int>(state.part_count));

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
