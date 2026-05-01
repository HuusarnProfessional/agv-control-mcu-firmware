#include "../initiated_request_handler_declarations.hpp"

#include "../../../middleware_parse_helpers.hpp"
#include "../../handler_helpers.hpp"

namespace initiated_request_handlers
{
    bool handle_get_path_chunk()
    {
        char mission_id[128] = {};
        char token_buffer[16] = {};
        bool ended = false;
        std::uint16_t part_number = 0;
        std::uint16_t chunk_number = 0;

        const bool mission_id_parsed_ok = middleware_parse_helpers::read_until_comma_or_end(mission_id, sizeof(mission_id), ended, 200000u);

        if (mission_id_parsed_ok == false)
        {
            return false;
        }

        if (ended == true)
        {
            return false;
        }

        const bool part_number_text_parsed_ok = middleware_parse_helpers::read_until_comma_or_end(token_buffer, sizeof(token_buffer), ended, 200000u);

        if (part_number_text_parsed_ok == false)
        {
            return false;
        }

        if ((ended == true) || (handler_helpers::parse_uint16(token_buffer, part_number) == false))
        {
            return false;
        }

        const bool chunk_number_text_parsed_ok = middleware_parse_helpers::read_until_comma_or_end(token_buffer, sizeof(token_buffer), ended, 200000u);

        if (chunk_number_text_parsed_ok == false)
        {
            return false;
        }

        if ((ended == false) || (handler_helpers::parse_uint16(token_buffer, chunk_number) == false))
        {
            return false;
        }

        return false;
    }
}
