#include "../incoming_request_handler_declarations.hpp"

#include <cstdio>

#include "../../../middleware_parse_helpers.hpp"
#include "../../handler_helpers.hpp"
#include "../../../../../control/primitives/command_speed/command_speed_state.hpp"

namespace
{
    constexpr std::uint32_t timeout_us = 50000u;
    constexpr std::size_t response_capacity = 32u;
}

namespace incoming_request_handlers
{
    bool handle_get_speed()
    {
        const bool parsed_ok = middleware_parse_helpers::read_end(timeout_us);

        if (parsed_ok == false)
        {
            return false;
        }

        const std::uint16_t requested_speed_mm_s = command_speed_state::get_requested_speed_mm_s();
        char response[response_capacity] = {};
        const int formatted_length = std::snprintf(
            response,
            sizeof(response),
            "rsp:speed(%u)",
            static_cast<unsigned int>(requested_speed_mm_s));

        if ((formatted_length <= 0) || (static_cast<std::size_t>(formatted_length) >= sizeof(response)))
        {
            return false;
        }

        return handler_helpers::write_response_text(response);
    }
}
