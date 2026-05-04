#include "../incoming_request_handler_declarations.hpp"

#include "../../../middleware_parse_helpers.hpp"
#include "../../handler_helpers.hpp"
#include "../../../../../control/primitives/command_speed/command_speed_state.hpp"

namespace
{
    constexpr std::uint32_t timeout_us = 50000u;
}

namespace incoming_request_handlers
{
    bool handle_set_speed()
    {
        std::uint16_t speed_mm_per_s = 0;
        const bool parsed_ok = middleware_parse_helpers::read_uint16_and_end(speed_mm_per_s, timeout_us);

        if (parsed_ok == false)
        {
            return false;
        }

        const bool speed_set = command_speed_state::set_requested_speed_mm_s(speed_mm_per_s);

        if (speed_set == false)
        {
            const bool fail_response_written = handler_helpers::write_response("rsp:fail(2)");

            if (fail_response_written == false)
            {
                return false;
            }

            return false;
        }

        const bool ok_response_written = handler_helpers::write_response("rsp:ok()");

        if (ok_response_written == false)
        {
            return false;
        }

        return true;
    }
}
