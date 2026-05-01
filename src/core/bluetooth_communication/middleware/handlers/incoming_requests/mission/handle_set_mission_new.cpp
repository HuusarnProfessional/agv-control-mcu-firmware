#include "../incoming_request_handler_declarations.hpp"

#include "../../../middleware_parse_helpers.hpp"

namespace incoming_request_handlers
{
    bool handle_set_mission_new()
    {
        char mission_id[128] = {};
        bool ended = false;
        std::uint16_t number_of_parts = 0;

        const bool mission_id_parsed_ok = middleware_parse_helpers::read_until_comma_or_end(mission_id, sizeof(mission_id), ended, 200000u);

        if (mission_id_parsed_ok == false)
        {
            return false;
        }

        if (ended == true)
        {
            return false;
        }

        const bool parsed_ok = middleware_parse_helpers::read_uint16_and_end(number_of_parts, 200000u);

        if (parsed_ok == false)
        {
            return false;
        }

        return false;
    }
}
