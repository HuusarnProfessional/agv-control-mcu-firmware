#include "../response_handler_declarations.hpp"

#include "../../../middleware_parse_helpers.hpp"

namespace response_handlers
{
    bool handle_speed()
    {
        std::int16_t speed_mm_per_s = 0;
        return middleware_parse_helpers::read_int16_and_end(speed_mm_per_s, 50000u);
    }
}
