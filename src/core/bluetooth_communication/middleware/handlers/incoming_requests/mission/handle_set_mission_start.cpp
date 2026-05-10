#include "../incoming_request_handler_declarations.hpp"

#include <Arduino.h>

#include "../../../middleware_parse_helpers.hpp"
#include "../../../../../mission/mission_runner.hpp"
#include "../../handler_helpers.hpp"

namespace
{
    constexpr std::uint32_t timeout_us = 200000u;
}

namespace incoming_request_handlers
{
    bool handle_set_mission_start()
    {
        char mission_id[128] = {};
        const bool parsed_ok = middleware_parse_helpers::read_csv_text_and_end(mission_id, sizeof(mission_id), timeout_us);

        if (parsed_ok == false)
        {
            Serial.println("mission rx set_mission_start parse fail");
            return false;
        }

        Serial.print("mission rx set_mission_start id=");
        Serial.println(mission_id);

        const mission_runner::runner_status runner_status = mission_runner::start_mission(mission_id);

        if (runner_status != mission_runner::runner_status::ok)
        {
            Serial.print("mission rx set_mission_start runner fail status=");
            Serial.println(static_cast<unsigned int>(runner_status));
            const bool fail_response_written = handler_helpers::write_response("rsp:fail(3)");

            if (fail_response_written == false)
            {
                return false;
            }

            return false;
        }

        Serial.println("mission tx rsp:ok()");
        const bool ok_response_written = handler_helpers::write_response("rsp:ok()");

        if (ok_response_written == false)
        {
            return false;
        }

        return true;
    }
}
