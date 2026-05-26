#include "../debug_handler_declarations.hpp"

#include <Arduino.h>

#include <cstddef>
#include <cstdint>
#include <cstdio>

#include "../../../middleware_parse_helpers.hpp"
#include "../../handler_helpers.hpp"
#include "../../../../../control/primitives/command_speed/command_speed_state.hpp"
#include "../../../../../control/primitives/motion_primitive/motion_primitive_status_monitor.hpp"
#include "../../../../../motion_mcu_communication/outgoing_payloads/service/drive_forward_payload.hpp"
#include "../../../../../position_sensorfusion/position_sensorfusion_pipeline.hpp"
#include "../../../../../position_trace/position_trace_logger.hpp"
#include "debug_handler_helpers.hpp"

namespace debug_handlers
{
    bool handle_run_entry_seeded_drive_forward_test()
    {
        std::int16_t distance_mm = 0;

        if (middleware_parse_helpers::read_int16_and_end(distance_mm, debug_handler_helpers::timeout_us) == false)
        {
            return debug_handler_helpers::write_bad_format();
        }

        if (distance_mm == 0)
        {
            return handler_helpers::write_response_text("err zero_distance");
        }

        const std::uint16_t requested_speed_mm_s = command_speed_state::get_requested_speed_mm_s();
        std::int32_t velocity_mm_s = static_cast<std::int32_t>(requested_speed_mm_s);

        if (distance_mm < 0)
        {
            velocity_mm_s = -velocity_mm_s;
        }

        const std::int64_t target_distance_um = static_cast<std::int64_t>(distance_mm) * 1000LL;

        position_trace_logger::clear();
        position_trace_logger::set_enabled(true);
        position_sensorfusion_pipeline::start_entry_seeded_drive_forward_test();

        if (drive_forward_payload::send(velocity_mm_s, target_distance_um) == false)
        {
            return handler_helpers::write_response_text("err drive_forward_send_failed");
        }

        motion_primitive_status_monitor::notify_drive_forward_sent(millis());

        char response[128] = {};
        const int length = std::snprintf(
            response,
            sizeof(response),
            "entry_seeded_drive_forward_test_started distance_mm %d",
            static_cast<int>(distance_mm));

        if ((length <= 0) || (static_cast<std::size_t>(length) >= sizeof(response)))
        {
            return false;
        }

        return handler_helpers::write_response_text(response);
    }
}
