#include "../incoming_request_handler_declarations.hpp"

#include "../../../middleware_parse_helpers.hpp"

namespace incoming_request_handlers
{
    bool handle_mission_part_info()
    {
        const bool parsed_ok = middleware_parse_helpers::discard_until_end(200000u);

        if (parsed_ok == false)
        {
            return false;
        }

        return false;
    }
}
