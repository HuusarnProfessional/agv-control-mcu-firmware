#include "../initiated_request_handler_declarations.hpp"

#include "../../../middleware_parse_helpers.hpp"

namespace initiated_request_handlers
{
    bool handle_get_mission_part()
    {
        char mission_id[128] = {};
        bool ended = false;
        std::uint16_t part_number = 0;

        const bool mission_id_parsed_ok = middleware_parse_helpers::read_until_comma_or_end(mission_id, sizeof(mission_id), ended, 200000u);

        if (mission_id_parsed_ok == false)
        {
            return false;
        }

        if (ended == true)
        {
            return false;
        }

        const bool parsed_ok = middleware_parse_helpers::read_uint16_and_end(part_number, 200000u);

        if (parsed_ok == false)
        {
            return false;
        }

        return false;
    }
}
