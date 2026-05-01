#include "../incoming_request_handler_declarations.hpp"

#include "../../../middleware_parse_helpers.hpp"

namespace incoming_request_handlers
{
    bool handle_set_watch_remove()
    {
        char command[128] = {};
        bool ended = false;
        std::uint16_t interval_ms = 0;

        const bool command_parsed_ok = middleware_parse_helpers::read_until_comma_or_end(command, sizeof(command), ended, 200000u);

        if (command_parsed_ok == false)
        {
            return false;
        }

        if (ended == true)
        {
            return false;
        }

        const bool parsed_ok = middleware_parse_helpers::read_uint16_and_end(interval_ms, 200000u);

        if (parsed_ok == false)
        {
            return false;
        }

        return false;
    }
}
