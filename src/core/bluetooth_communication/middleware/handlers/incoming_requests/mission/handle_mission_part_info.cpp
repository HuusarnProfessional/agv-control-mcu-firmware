#include "../incoming_request_handler_declarations.hpp"

#include "../../../middleware_parse_helpers.hpp"
#include "../../../../mission/mission_transfer.hpp"
#include "../../handler_helpers.hpp"

namespace
{
    constexpr std::uint32_t timeout_us = 200000u;
}

namespace incoming_request_handlers
{
    bool handle_mission_part_info()
    {
        char mission_id[128] = {};
        char token_buffer[16] = {};
        char start_command[128] = {};
        char end_command[128] = {};
        bool ended = false;
        std::uint16_t part_number = 0u;
        std::uint16_t path_chunk_count = 0u;

        const bool mission_id_parsed_ok = middleware_parse_helpers::read_until_comma_or_end(mission_id, sizeof(mission_id), ended, timeout_us);

        if (mission_id_parsed_ok == false)
        {
            return false;
        }

        if (ended == true)
        {
            return false;
        }

        const bool part_number_text_parsed_ok = middleware_parse_helpers::read_until_comma_or_end(token_buffer, sizeof(token_buffer), ended, timeout_us);

        if (part_number_text_parsed_ok == false)
        {
            return false;
        }

        if ((ended == true) || (handler_helpers::parse_uint16(token_buffer, part_number) == false))
        {
            return false;
        }

        const bool start_command_parsed_ok = middleware_parse_helpers::read_until_comma_or_end(start_command, sizeof(start_command), ended, timeout_us);

        if (start_command_parsed_ok == false)
        {
            return false;
        }

        if (ended == true)
        {
            return false;
        }

        const bool end_command_parsed_ok = middleware_parse_helpers::read_until_comma_or_end(end_command, sizeof(end_command), ended, timeout_us);

        if (end_command_parsed_ok == false)
        {
            return false;
        }

        if (ended == true)
        {
            return false;
        }

        const bool parsed_ok = middleware_parse_helpers::read_uint16_and_end(path_chunk_count, timeout_us);

        if (parsed_ok == false)
        {
            return false;
        }

        const mission_transfer::transfer_status transfer_status = mission_transfer::set_part_info(
            mission_id,
            part_number,
            start_command,
            end_command,
            path_chunk_count);

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
