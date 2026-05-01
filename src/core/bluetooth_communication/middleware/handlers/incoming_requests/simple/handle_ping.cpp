#include "../incoming_request_handler_declarations.hpp"

#include "../../../middleware_parse_helpers.hpp"
#include "../../handler_helpers.hpp"

namespace incoming_request_handlers
{
    bool handle_ping()
    {
        const bool parsed_ok = middleware_parse_helpers::read_end(50000u);

        if (parsed_ok == false)
        {
            return false;
        }

        return handler_helpers::write_response("rsp:pong()");
    }
}
