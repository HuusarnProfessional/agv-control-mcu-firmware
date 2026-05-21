#include "motion_primitive_status_payload.hpp"

#include <Arduino.h>

#include "../../payload_helper_functions.hpp"
#include "../../state/incoming/incoming_state.hpp"

namespace motion_primitive_status_payload
{
    void handle(const std::uint8_t *payload_data, std::uint8_t payload_length)
    {
        motion_mcu_incoming_state::motion_primitive_status_state state = {};
        state.received_time_ms = millis();

        const bool has_command_id = payload_helper_functions::read_u32_le(payload_data, payload_length, 0U, state.command_id);
        const bool has_primitive_id = payload_helper_functions::read_u8(payload_data, payload_length, 4U, state.active_primitive_id);
        const bool has_state = payload_helper_functions::read_u8(payload_data, payload_length, 5U, state.state);
        const bool has_failure_code = payload_helper_functions::read_u8(payload_data, payload_length, 6U, state.failure_code);
        const bool has_status_time = payload_helper_functions::read_u32_le(payload_data, payload_length, 8U, state.status_time_ms);

        if ((has_command_id == true) &&
            (has_primitive_id == true) &&
            (has_state == true) &&
            (has_failure_code == true) &&
            (has_status_time == true))
        {
            state.valid = true;
            motion_mcu_incoming_state::set_motion_primitive_status(state);
        }
    }
}
