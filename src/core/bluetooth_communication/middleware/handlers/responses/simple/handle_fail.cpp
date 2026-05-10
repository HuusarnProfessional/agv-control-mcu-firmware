#include "../response_handler_declarations.hpp"

#include <Arduino.h>

#include "../../../middleware_parse_helpers.hpp"
#include "../../../../../mission/mission_transfer.hpp"

namespace response_handlers
{
    bool handle_fail()
    {
        constexpr std::uint32_t timeout_us = 50000u;
        std::uint8_t error_code = 0;
        const bool parsed_ok = middleware_parse_helpers::read_uint8_and_end(error_code, timeout_us);

        if (parsed_ok == false)
        {
            return false;
        }

        Serial.print("mission rx rsp:fail(");
        Serial.print(static_cast<unsigned int>(error_code));
        Serial.println(")");
        mission_transfer::handle_response_fail(error_code);

        return true;
    }
}
