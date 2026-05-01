#include "../response_handler_declarations.hpp"

#include "../../../middleware_parse_helpers.hpp"

namespace response_handlers
{
    bool handle_fail()
    {
        std::uint8_t error_code = 0;
        return middleware_parse_helpers::read_uint8_and_end(error_code, 50000u);
    }
}
