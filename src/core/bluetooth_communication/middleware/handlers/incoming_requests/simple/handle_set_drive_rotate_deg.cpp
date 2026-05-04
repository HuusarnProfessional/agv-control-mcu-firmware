#include "../incoming_request_handler_declarations.hpp"

#include "../../../middleware_parse_helpers.hpp"
#include "../../handler_helpers.hpp"
#include "../../../../../control/primitives/command_speed/command_speed_state.hpp"
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
            return false;
        }

        if (delta_heading == 0)
        {
            return handler_helpers::write_response("rsp:ok()");
        }

        const std::uint16_t requested_speed = command_speed_state::get_requested_speed_mm_s();
        const std::int64_t target_rotation_urad = static_cast<std::int64_t>(delta_heading) * urad_per_deg;
        std::int32_t yaw_rate_mdeg_s = static_cast<std::int32_t>(requested_speed) * mdeg_per_deg;

        if (delta_heading < 0)
        {
            yaw_rate_mdeg_s = -yaw_rate_mdeg_s;
        }

        const bool command_sent = rotate_delta_payload::send(0, yaw_rate_mdeg_s, target_rotation_urad, false, 0, 0);

        if (command_sent == false)
        {
            const bool fail_response_written = handler_helpers::write_response("rsp:fail(0)");

            if (fail_response_written == false)
            {
                return false;
            }

            return false;
        }

        const bool ok_response_written = handler_helpers::write_response("rsp:ok()");

        if (ok_response_written == false)
        {
            return false;
        }

        return true;
    }
}
