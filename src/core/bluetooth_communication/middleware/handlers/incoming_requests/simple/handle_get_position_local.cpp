#include "../incoming_request_handler_declarations.hpp"

#include "../../../middleware_parse_helpers.hpp"

namespace incoming_request_handlers
{
    bool handle_get_position_local()
    {
        const bool parsed_ok = middleware_parse_helpers::read_end(50000u);

        if (parsed_ok == false)
        {
            return false;
        }

        return false;
    }
}
