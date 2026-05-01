#include "../incoming_request_handler_declarations.hpp"

#include "../../../middleware_parse_helpers.hpp"
#include "../../handler_helpers.hpp"

namespace incoming_request_handlers
{
    bool handle_ping()
    {
        constexpr std::uint32_t timeout_us = 50000u;
        const bool parsed_ok = middleware_parse_helpers::read_end(timeout_us);

        if (parsed_ok == false)
        {
            return false;
        }

        const bool response_written = handler_helpers::write_response("rsp:pong()");

        if (response_written == false)
        {
            return false;
        }

        return true;
    }
}
