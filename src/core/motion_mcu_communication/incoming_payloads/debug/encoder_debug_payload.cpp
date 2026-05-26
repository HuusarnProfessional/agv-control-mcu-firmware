#include "encoder_debug_payload.hpp"

#include <Arduino.h>

#include <cstddef>

#include "../../payload_helper_functions.hpp"
#include "../../state/debug/debug_state.hpp"

namespace encoder_debug_payload
{
    void handle(const std::uint8_t *payload_data, std::uint8_t payload_length)
    {
        motion_mcu_debug_state::encoder_debug_state state = motion_mcu_debug_state::get_encoder_debug();
        state.valid = false;
        state.received_time_ms = millis();

        if (payload_helper_functions::read_u8(payload_data, payload_length, 0U, state.count) == false)
        {
            return;
        }

        if (state.count > motion_mcu_debug_state::encoder_capacity)
        {
            return;
        }

        std::size_t offset = 1U;

        for (std::uint8_t index = 0U; index < state.count; ++index)
        {
            const bool has_raw = payload_helper_functions::read_u16_le(payload_data, payload_length, offset, state.raw[index]);
            const bool has_angle_mdeg = payload_helper_functions::read_u32_le(payload_data, payload_length, offset + 2U, state.angle_mdeg[index]);
            const bool has_time_ms = payload_helper_functions::read_u32_le(payload_data, payload_length, offset + 6U, state.time_ms[index]);
            const bool has_status = payload_helper_functions::read_u8(payload_data, payload_length, offset + 10U, state.status[index]);

            if ((has_raw == false) || (has_angle_mdeg == false) || (has_time_ms == false) || (has_status == false))
            {
                return;
            }

            offset += 11U;
        }

        state.valid = true;
        motion_mcu_debug_state::set_encoder_debug(state);
    }
}
