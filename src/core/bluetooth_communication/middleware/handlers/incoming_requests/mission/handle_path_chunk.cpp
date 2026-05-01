#include "../incoming_request_handler_declarations.hpp"

#include "../../../middleware_parse_helpers.hpp"
#include "../../handler_helpers.hpp"

namespace incoming_request_handlers
{
    bool handle_path_chunk()
    {
        char mission_id[128] = {};
        char token_buffer[16] = {};
        bool ended = false;
        std::uint16_t part_number = 0;
        std::uint16_t chunk_number = 0;
        std::uint16_t point_count = 0;

        const bool mission_id_parsed_ok = middleware_parse_helpers::read_until_comma_or_end(mission_id, sizeof(mission_id), ended, 500000u);

        if (mission_id_parsed_ok == false)
        {
            return false;
        }

        if (ended == true)
        {
            return false;
        }

        const bool part_number_text_parsed_ok = middleware_parse_helpers::read_until_comma_or_end(token_buffer, sizeof(token_buffer), ended, 500000u);

        if (part_number_text_parsed_ok == false)
        {
            return false;
        }

        if ((ended == true) || (handler_helpers::parse_uint16(token_buffer, part_number) == false))
        {
            return false;
        }

        const bool chunk_number_text_parsed_ok = middleware_parse_helpers::read_until_comma_or_end(token_buffer, sizeof(token_buffer), ended, 500000u);

        if (chunk_number_text_parsed_ok == false)
        {
            return false;
        }

        if ((ended == true) || (handler_helpers::parse_uint16(token_buffer, chunk_number) == false))
        {
            return false;
        }

        const bool point_count_text_parsed_ok = middleware_parse_helpers::read_until_comma_or_end(token_buffer, sizeof(token_buffer), ended, 500000u);

        if (point_count_text_parsed_ok == false)
        {
            return false;
        }

        if ((ended == true) || (handler_helpers::parse_uint16(token_buffer, point_count) == false))
        {
            return false;
        }

        const std::size_t binary_length = static_cast<std::size_t>(point_count) * 4u;

        for (std::size_t index = 0; index < binary_length; ++index)
        {
            std::uint8_t discarded_byte = 0;

            const bool binary_read_ok = middleware_parse_helpers::read_binary(&discarded_byte, 1u, 500000u);

            if (binary_read_ok == false)
            {
                return false;
            }
        }

        const bool parsed_ok = middleware_parse_helpers::read_end(500000u);

        if (parsed_ok == false)
        {
            return false;
        }

        return false;
    }
}
