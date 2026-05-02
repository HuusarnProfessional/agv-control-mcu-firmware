#include "../response_handler_declarations.hpp"

#include "../../../middleware_parse_helpers.hpp"
#include "../../../../../mission/mission_transfer.hpp"

namespace response_handlers
{
    bool handle_ok()
    {
        constexpr std::uint32_t timeout_us = 50000u;
        const bool parsed_ok = middleware_parse_helpers::read_end(timeout_us);

        if (parsed_ok == false)
        {
            return false;
        }

        mission_transfer::handle_response_ok();

        return true;
    }
}
