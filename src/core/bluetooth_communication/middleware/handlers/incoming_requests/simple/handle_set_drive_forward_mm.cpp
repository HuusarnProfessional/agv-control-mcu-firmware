#include "../incoming_request_handler_declarations.hpp"

#include "../../../middleware_parse_helpers.hpp"

namespace incoming_request_handlers
{
    bool handle_set_drive_forward_mm()
    {
        std::int16_t distance_mm = 0;
        const bool parsed_ok = middleware_parse_helpers::read_int16_and_end(distance_mm, 50000u);

        if (parsed_ok == false)
        {
            return false;
        }

        return false;
    }
}
