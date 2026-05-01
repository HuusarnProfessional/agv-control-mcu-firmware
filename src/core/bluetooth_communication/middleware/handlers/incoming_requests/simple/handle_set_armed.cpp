#include "../incoming_request_handler_declarations.hpp"

#include "../../../middleware_parse_helpers.hpp"

namespace incoming_request_handlers
{
    bool handle_set_armed()
    {
        bool is_armed = false;
        const bool parsed_ok = middleware_parse_helpers::read_bool_and_end(is_armed, 50000u);

        if (parsed_ok == false)
        {
            return false;
        }

        return false;
    }
}
