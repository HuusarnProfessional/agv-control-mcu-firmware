#include "../response_handler_declarations.hpp"

#include "../../../middleware_parse_helpers.hpp"

namespace response_handlers
{
    bool handle_ok()
    {
        return middleware_parse_helpers::read_end(50000u);
    }
}
