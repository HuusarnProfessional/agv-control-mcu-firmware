#include "../incoming_request_handler_declarations.hpp"

#include "../../../middleware_parse_helpers.hpp"
#include "../../../../../mission/mission_transfer.hpp"
#include "../../handler_helpers.hpp"

namespace
{
    constexpr std::uint32_t timeout_us = 200000u;
}

namespace incoming_request_handlers
{
    bool handle_set_mission_new()
    {
        char mission_id[128] = {};
        bool ended = false;
        std::uint16_t number_of_parts = 0;

        const bool mission_id_parsed_ok = middleware_parse_helpers::read_until_comma_or_end(mission_id, sizeof(mission_id), ended, timeout_us);

        if (mission_id_parsed_ok == false)
        {
            return false;
        }

        if (ended == true)
        {
            return false;
        }

        const bool parsed_ok = middleware_parse_helpers::read_uint16_and_end(number_of_parts, timeout_us);

        if (parsed_ok == false)
        {
            return false;
        }

        const mission_transfer::transfer_status transfer_status = mission_transfer::begin_new_mission(mission_id, number_of_parts);

        if (transfer_status != mission_transfer::transfer_status::ok)
        {
            const bool fail_response_written = handler_helpers::write_response("rsp:fail(3)");

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
