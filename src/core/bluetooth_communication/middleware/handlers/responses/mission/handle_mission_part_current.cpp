#include "../response_handler_declarations.hpp"

#include "../../../middleware_parse_helpers.hpp"

namespace response_handlers
{
    bool handle_mission_part_current()
    {
        std::uint16_t part_number = 0;
        return middleware_parse_helpers::read_uint16_and_end(part_number, 50000u);
    }
}
