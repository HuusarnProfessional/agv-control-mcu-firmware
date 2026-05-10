#include "../incoming_request_handler_declarations.hpp"

#include <Arduino.h>

#include "../../../middleware_parse_helpers.hpp"
#include "../../handler_helpers.hpp"
#include "../../../../../control/primitives/command_speed/command_speed_state.hpp"
#include "../../../../../control/primitives/motion_primitive/motion_primitive_status_monitor.hpp"
#include "../../../../../motion_mcu_communication/outgoing_payloads/service/rotate_delta_payload.hpp"

namespace
{
    constexpr std::uint32_t timeout_us = 50000u;
    constexpr std::int64_t urad_per_deg = 17453LL;
    constexpr std::int32_t mdeg_per_deg = 1000;
}

namespace incoming_request_handlers
{
    bool handle_set_drive_rotate_deg()
    {
        std::int16_t delta_heading = 0;
        const bool parsed_ok = middleware_parse_helpers::read_int16_and_end(delta_heading, timeout_us);

        if (parsed_ok == false)
        {
            Serial.println("bt rotate_deg parse fail");
            return false;
        }

        Serial.print("bt rotate_deg delta_heading=");
        Serial.println(static_cast<int>(delta_heading));

        if (delta_heading == 0)
        {
            Serial.println("bt rotate_deg zero delta -> rsp:ok()");
            return handler_helpers::write_response("rsp:ok()");
        }

        const std::uint16_t requested_speed = command_speed_state::get_requested_speed_mm_s();
        const std::int64_t target_rotation_urad = static_cast<std::int64_t>(delta_heading) * urad_per_deg;
        std::int32_t yaw_rate_mdeg_s = static_cast<std::int32_t>(requested_speed) * mdeg_per_deg;

        if (delta_heading < 0)
        {
            yaw_rate_mdeg_s = -yaw_rate_mdeg_s;
        }

        Serial.print("bt rotate_deg requested_speed_mm_s=");
        Serial.println(static_cast<unsigned int>(requested_speed));
        Serial.print("bt rotate_deg target_rotation_urad=");
        Serial.println(static_cast<long long>(target_rotation_urad));
        Serial.print("bt rotate_deg yaw_rate_mdeg_s=");
        Serial.println(static_cast<long>(yaw_rate_mdeg_s));
        Serial.println("bt rotate_deg send -> stm");

        const bool command_sent = rotate_delta_payload::send(0, yaw_rate_mdeg_s, target_rotation_urad, false, 0, 0);

        if (command_sent == false)
        {
            Serial.println("bt rotate_deg stm send fail");
            const bool fail_response_written = handler_helpers::write_response("rsp:fail(0)");

            if (fail_response_written == false)
            {
                return false;
            }

            return false;
        }

        Serial.println("bt rotate_deg stm send ok");
        motion_primitive_status_monitor::notify_rotate_delta_sent(millis());
        const bool ok_response_written = handler_helpers::write_response("rsp:ok()");

        if (ok_response_written == false)
        {
            return false;
        }

        return true;
    }
}
