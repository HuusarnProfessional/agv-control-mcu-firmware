#include "../debug_handler_declarations.hpp"

#include <cstddef>
#include <cstdint>
#include <cstdio>

#include "../../../middleware_parse_helpers.hpp"
#include "../../handler_helpers.hpp"
#include "../../../../../motion_mcu_communication/heartbeat/motion_mcu_heartbeat.hpp"
#include "../../../../../motion_mcu_communication/state/incoming/incoming_state.hpp"

namespace
{
    constexpr std::uint32_t timeout_us = 50000U;
    constexpr std::size_t response_capacity = 96U;
}

namespace debug_handlers
{
    bool handle_get_status()
    {
        if (middleware_parse_helpers::read_end(timeout_us) == false)
        {
            return false;
        }

        const motion_mcu_heartbeat::snapshot heartbeat = motion_mcu_heartbeat::read_snapshot();
        const motion_mcu_incoming_state::safety_status_state safety_status = motion_mcu_incoming_state::get_safety_status();
        const motion_mcu_incoming_state::power_status_state power_status = motion_mcu_incoming_state::get_power_status();
        const char *stm_status = ((heartbeat.has_seen_packet == true) && (heartbeat.packet_timed_out == false)) ? "ok" : "stale";

        char response[response_capacity] = {};
        const int formatted_length = std::snprintf(
            response,
            sizeof(response),
            "status stm %s fault %u voltage_mv %lu",
            stm_status,
            safety_status.fault_latched ? 1U : 0U,
            static_cast<unsigned long>(power_status.voltage_mv));

        if ((formatted_length <= 0) || (static_cast<std::size_t>(formatted_length) >= sizeof(response)))
        {
            return false;
        }

        return handler_helpers::write_response_text(response);
    }
}
