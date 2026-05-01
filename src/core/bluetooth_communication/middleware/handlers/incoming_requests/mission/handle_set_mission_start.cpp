#include "../incoming_request_handler_declarations.hpp"

#include "../../../middleware_parse_helpers.hpp"

namespace incoming_request_handlers
{
    bool handle_set_mission_start()
    {
        char mission_id[128] = {};
        const bool parsed_ok = middleware_parse_helpers::read_csv_text_and_end(mission_id, sizeof(mission_id), 200000u);

        if (parsed_ok == false)
        {
            return false;
        }

        return false;
    }
}
