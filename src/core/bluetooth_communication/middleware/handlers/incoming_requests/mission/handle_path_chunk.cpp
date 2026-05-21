#include "../incoming_request_handler_declarations.hpp"

#include <cstddef>

#include "../../../middleware_parse_helpers.hpp"
#include "../../../../../mission/mission_buffer.hpp"
#include "../../../../../mission/mission_transfer.hpp"
#include "../../handler_helpers.hpp"

namespace
{
    constexpr std::uint32_t timeout_us = 500000u;
}

namespace incoming_request_handlers
{
    bool handle_path_chunk()
    {
        char mission_id[128] = {};
        char token_buffer[16] = {};
        std::uint8_t path_data[mission_buffer::max_chunk_data_length] = {};
        bool ended = false;
        std::uint16_t part_number = 0u;
        std::uint16_t chunk_number = 0u;
        std::uint16_t point_count = 0u;

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

        const bool chunk_number_text_parsed_ok = middleware_parse_helpers::read_until_comma_or_end(token_buffer, sizeof(token_buffer), ended, timeout_us);

        if (chunk_number_text_parsed_ok == false)
        {
            return false;
        }

        if ((ended == true) || (handler_helpers::parse_uint16(token_buffer, chunk_number) == false))
        {
            return false;
        }

        const bool point_count_text_parsed_ok = middleware_parse_helpers::read_until_comma_or_end(token_buffer, sizeof(token_buffer), ended, timeout_us);

        if (point_count_text_parsed_ok == false)
        {
            return false;
        }

        if ((ended == true) || (handler_helpers::parse_uint16(token_buffer, point_count) == false))
        {
            return false;
        }

        const std::size_t binary_length = static_cast<std::size_t>(point_count) * 4u;

        if (binary_length > sizeof(path_data))
        {
            return false;
        }

        const bool binary_read_ok = middleware_parse_helpers::read_binary(path_data, binary_length, timeout_us);

        if (binary_read_ok == false)
        {
            return false;
        }

        const bool parsed_ok = middleware_parse_helpers::read_end(timeout_us);

        if (parsed_ok == false)
        {
            return false;
        }

        const mission_transfer::transfer_status transfer_status = mission_transfer::append_path_chunk(
            mission_id,
            part_number,
            chunk_number,
            point_count,
            path_data,
            binary_length);

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
