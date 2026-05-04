#include "../incoming_request_handler_declarations.hpp"

#include "../../../middleware_parse_helpers.hpp"
#include "../../handler_helpers.hpp"
#include "../../../../../control/primitives/command_speed/command_speed_state.hpp"
#include "../../../../../motion_mcu_communication/outgoing_payloads/service/drive_forward_payload.hpp"

namespace
{
    constexpr std::uint32_t timeout_us = 50000u;
}

namespace incoming_request_handlers
{
    bool handle_set_drive_forward_mm()
    {
        std::int16_t distance_mm = 0;
        const bool parsed_ok = middleware_parse_helpers::read_int16_and_end(distance_mm, timeout_us);

        if (parsed_ok == false)
        {
            return false;
        }

        if (distance_mm == 0)
        {
            return handler_helpers::write_response("rsp:ok()");
        }

        const std::uint16_t requested_speed_mm_s = command_speed_state::get_requested_speed_mm_s();
        std::int32_t velocity_mm_s = 0;

        if (distance_mm > 0)
        {
            velocity_mm_s = static_cast<std::int32_t>(requested_speed_mm_s);
        }
        else
        {
            velocity_mm_s = -static_cast<std::int32_t>(requested_speed_mm_s);
        }

        const std::int64_t target_distance_um = static_cast<std::int64_t>(distance_mm) * 1000LL;
        const bool command_sent = drive_forward_payload::send(velocity_mm_s, target_distance_um);

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
