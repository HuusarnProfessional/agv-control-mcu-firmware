#include "../debug_handler_declarations.hpp"

#include <cstddef>
#include <cstdio>

#include "../../../middleware_parse_helpers.hpp"
#include "../../handler_helpers.hpp"
#include "../../../../../motion_mcu_communication/motion_mcu_routes.hpp"
#include "../../../../../motion_mcu_communication/state/debug/debug_state.hpp"
#include "debug_handler_helpers.hpp"

namespace debug_handlers
{
    bool handle_get_voltage_debug()
    {
        if (middleware_parse_helpers::read_end(debug_handler_helpers::timeout_us) == false)
        {
            return debug_handler_helpers::write_bad_format();
        }

        const std::uint8_t payload_id = static_cast<std::uint8_t>(motion_mcu_routes::incoming_payload_id::voltage_debug);

        if (motion_mcu_debug_state::is_stream_enabled(payload_id) == false)
        {
            return debug_handler_helpers::write_stream_not_active("voltage_debug");
        }

        const motion_mcu_debug_state::voltage_debug_state state = motion_mcu_debug_state::get_voltage_debug();

        if (state.valid == false)
        {
            return debug_handler_helpers::write_missing_data("voltage_debug");
        }

        char response[128] = {};
        const int length = std::snprintf(
            response,
            sizeof(response),
            "voltage_debug raw_adc %u voltage_mv %lu time_ms %lu status %u",
            static_cast<unsigned>(state.raw_adc),
            static_cast<unsigned long>(state.voltage_mv),
            static_cast<unsigned long>(state.time_ms),
            static_cast<unsigned>(state.status));

        if ((length <= 0) || (static_cast<std::size_t>(length) >= sizeof(response)))
        {
            return false;
        }

        return handler_helpers::write_response_text(response);
    }
}
