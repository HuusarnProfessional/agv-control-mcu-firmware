#include "../debug_handler_declarations.hpp"

#include "../../../middleware_parse_helpers.hpp"
#include "../../handler_helpers.hpp"

namespace debug_handlers
{
    bool handle_debug_command()
    {
        constexpr std::uint32_t timeout_us = 50000u;
        const bool parsed_ok = middleware_parse_helpers::discard_until_end(timeout_us);

        if (parsed_ok == false)
        {
            return false;
        }

        return handler_helpers::write_response("rsp:fail(1)");
    }
}
