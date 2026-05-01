#include "../response_handler_declarations.hpp"

#include "../../../middleware_parse_helpers.hpp"

namespace response_handlers
{
    bool handle_armed()
    {
        bool is_armed = false;
        return middleware_parse_helpers::read_bool_and_end(is_armed, 50000u);
    }
}
