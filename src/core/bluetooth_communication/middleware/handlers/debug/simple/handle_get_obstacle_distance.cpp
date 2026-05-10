#include "../debug_handler_declarations.hpp"

#include <cstddef>
#include <cstdint>
#include <cstdio>

#include "../../../middleware_parse_helpers.hpp"
#include "../../handler_helpers.hpp"
#include "../../../../../motion_mcu_communication/motion_mcu_routes.hpp"
#include "../../../../../motion_mcu_communication/state/debug/debug_state.hpp"
#include "debug_handler_helpers.hpp"

namespace debug_handlers
{
    bool handle_get_obstacle_distance()
    {
        char token_buffer[8] = {};
        bool ended = false;

        if (middleware_parse_helpers::read_until_comma_or_end(token_buffer, sizeof(token_buffer), ended, debug_handler_helpers::timeout_us) == false)
        {
            return debug_handler_helpers::write_bad_format();
        }

        if (ended == false)
        {
            return debug_handler_helpers::write_bad_format();
        }

        const std::uint8_t payload_id = static_cast<std::uint8_t>(motion_mcu_routes::incoming_payload_id::obstacle_debug);

        if (motion_mcu_debug_state::is_stream_enabled(payload_id) == false)
        {
            return debug_handler_helpers::write_stream_not_active("obstacle_debug");
        }

        const motion_mcu_debug_state::obstacle_debug_state state = motion_mcu_debug_state::get_obstacle_debug();

        if ((state.valid == false) || (state.count == 0U))
        {
            return debug_handler_helpers::write_missing_data("obstacle_debug");
        }

        std::uint8_t sensor_id = 0U;

        if (token_buffer[0] != '\0')
        {
            if (handler_helpers::parse_uint8(token_buffer, sensor_id) == false)
            {
                return debug_handler_helpers::write_bad_format();
            }

            if (sensor_id >= state.count)
            {
                return debug_handler_helpers::write_missing_data("obstacle");
            }

            char response[64] = {};
            const int length = std::snprintf(
                response,
                sizeof(response),
                "distance %u %lu",
                static_cast<unsigned>(sensor_id),
                static_cast<unsigned long>(state.distance_mm[sensor_id]));

            if ((length <= 0) || (static_cast<std::size_t>(length) >= sizeof(response)))
            {
                return false;
            }

            return handler_helpers::write_response_text(response);
        }

        char response[64] = {};
        const int length = std::snprintf(
            response,
            sizeof(response),
            "distance %lu",
            static_cast<unsigned long>(state.distance_mm[0]));

        if ((length <= 0) || (static_cast<std::size_t>(length) >= sizeof(response)))
        {
            return false;
        }

        return handler_helpers::write_response_text(response);
    }
}
