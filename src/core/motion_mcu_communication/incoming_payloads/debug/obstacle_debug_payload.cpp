#include "obstacle_debug_payload.hpp"

#include <Arduino.h>

#include "../../payload_helper_functions.hpp"
#include "../../state/debug/debug_state.hpp"

namespace obstacle_debug_payload
{
    void handle(const std::uint8_t *payload_data, std::uint8_t payload_length)
    {
        motion_mcu_debug_state::obstacle_debug_state state = motion_mcu_debug_state::get_obstacle_debug();
        state.valid = false;
        state.received_time_ms = millis();

        if (payload_helper_functions::read_u8(payload_data, payload_length, 0U, state.count) == false)
        {
            return;
        }

        if (state.count > motion_mcu_debug_state::obstacle_capacity)
        {
            return;
        }

        std::size_t offset = 1U;

        for (std::uint8_t index = 0U; index < state.count; ++index)
        {
            const bool has_distance_mm = payload_helper_functions::read_u32_le(payload_data, payload_length, offset, state.distance_mm[index]);
            const bool has_time_ms = payload_helper_functions::read_u32_le(payload_data, payload_length, offset + 4U, state.time_ms[index]);
            const bool has_status = payload_helper_functions::read_u8(payload_data, payload_length, offset + 8U, state.status[index]);

            if ((has_distance_mm == false) || (has_time_ms == false) || (has_status == false))
            {
                return;
            }

            offset += 9U;
        }

        state.valid = true;
        motion_mcu_debug_state::set_obstacle_debug(state);
    }
}
