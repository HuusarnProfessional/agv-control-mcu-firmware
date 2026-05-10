#include "../response_handler_declarations.hpp"

#include <Arduino.h>

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

        Serial.println("mission rx rsp:ok()");
        mission_transfer::handle_response_ok();

        return true;
    }
}
