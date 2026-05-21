#include "../incoming_request_handler_declarations.hpp"

#include <cstddef>

#include "../../../middleware_parse_helpers.hpp"
#include "../../../route/middleware_route_table.hpp"
#include "../../../../../mission/mission_transfer.hpp"
#include "../../handler_helpers.hpp"

namespace
{
    constexpr std::uint32_t timeout_us = 200000u;

    bool copy_text(char *destination, std::size_t capacity, const char *source)
    {
        if ((destination == nullptr) || (capacity == 0u) || (source == nullptr))
        {
            return false;
        }

        std::size_t index = 0u;

        while (source[index] != '\0')
        {
            if ((index + 1u) >= capacity)
            {
                return false;
            }

            destination[index] = source[index];
            ++index;
        }

        destination[index] = '\0';
        return true;
    }

    bool parse_stored_command(const char *command_text, mission_buffer::mission_command_view &command_out)
    {
        if (command_text == nullptr)
        {
            return false;
        }

        command_out.route = nullptr;
        command_out.argument_stream[0] = '\0';

        std::size_t command_name_length = 0u;

        while ((command_text[command_name_length] != '\0') && (command_text[command_name_length] != '('))
        {
            ++command_name_length;
        }

        if ((command_name_length == 0u) || (command_text[command_name_length] != '('))
        {
            return false;
        }

        char command_name[mission_buffer::max_command_length] = {};

        for (std::size_t index = 0u; index < command_name_length; ++index)
        {
            if ((index + 1u) >= sizeof(command_name))
            {
                return false;
            }

            command_name[index] = command_text[index];
        }

        command_name[command_name_length] = '\0';

        const middleware_route_types::middleware_command_route *route =
            middleware_route_table::find_incoming_request_route(command_name);

        if (route == nullptr)
        {
            return false;
        }

        if (copy_text(command_out.argument_stream, sizeof(command_out.argument_stream), &command_text[command_name_length + 1u]) == false)
        {
            return false;
        }

        command_out.route = route;
        return true;
    }
}

namespace incoming_request_handlers
{
    bool handle_mission_part_info()
    {
        char mission_id[128] = {};
        char token_buffer[16] = {};
        char start_command[128] = {};
        char end_command[128] = {};
        mission_buffer::mission_command_view start_command_view = {};
        mission_buffer::mission_command_view end_command_view = {};
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

        const bool start_command_parsed_ok = middleware_parse_helpers::read_command_until_comma_or_end(start_command, sizeof(start_command), ended, timeout_us);

        if (start_command_parsed_ok == false)
        {
            return false;
        }

        if (ended == true)
        {
            return false;
        }

        if (parse_stored_command(start_command, start_command_view) == false)
        {
            return false;
        }

        const bool end_command_parsed_ok = middleware_parse_helpers::read_command_until_comma_or_end(end_command, sizeof(end_command), ended, timeout_us);

        if (end_command_parsed_ok == false)
        {
            return false;
        }

        if (ended == true)
        {
            return false;
        }

        if (parse_stored_command(end_command, end_command_view) == false)
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
            start_command_view,
            end_command_view,
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

        const bool transfer_complete = mission_transfer::is_transfer_complete();

        if (transfer_complete == false)
        {
            return true;
        }

        const bool ready_response_written = handler_helpers::write_mission_ready_response(mission_id);

        if (ready_response_written == false)
        {
            return false;
        }

        return true;
    }
}
