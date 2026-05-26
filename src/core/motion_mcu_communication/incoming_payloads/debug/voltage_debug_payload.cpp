#include "voltage_debug_payload.hpp"

#include <Arduino.h>

#include "../../payload_helper_functions.hpp"
#include "../../state/debug/debug_state.hpp"

namespace voltage_debug_payload
{
    void handle(const std::uint8_t *payload_data, std::uint8_t payload_length)
    {
        motion_mcu_debug_state::voltage_debug_state state = motion_mcu_debug_state::get_voltage_debug();
        state.valid = false;
        state.received_time_ms = millis();

        const bool has_raw_adc = payload_helper_functions::read_u16_le(payload_data, payload_length, 0U, state.raw_adc);
        const bool has_voltage_mv = payload_helper_functions::read_u32_le(payload_data, payload_length, 2U, state.voltage_mv);
        const bool has_time_ms = payload_helper_functions::read_u32_le(payload_data, payload_length, 6U, state.time_ms);
        const bool has_status = payload_helper_functions::read_u8(payload_data, payload_length, 10U, state.status);

        if ((has_raw_adc == false) || (has_voltage_mv == false) || (has_time_ms == false) || (has_status == false))
        {
            return;
        }

        state.valid = true;
        motion_mcu_debug_state::set_voltage_debug(state);
    }
}
