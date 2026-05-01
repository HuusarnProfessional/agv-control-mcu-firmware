#include "../incoming_request_handler_declarations.hpp"

#include "../../../middleware_parse_helpers.hpp"

namespace incoming_request_handlers
{
    bool handle_set_watch_keep_alive()
    {
        std::uint16_t timeout_ms = 0;
        const bool parsed_ok = middleware_parse_helpers::read_uint16_and_end(timeout_ms, 200000u);

        if (parsed_ok == false)
        {
            return false;
        }

        return false;
    }
}
