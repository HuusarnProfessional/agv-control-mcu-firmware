#include "safety_status_payload.hpp"

#include "../../payload_helper_functions.hpp"
#include "../../state/incoming/incoming_state.hpp"

namespace safety_status_payload
{
    void handle(const std::uint8_t *payload_data, std::uint8_t payload_length)
    {
        motion_mcu_incoming_state::safety_status_state state = {};
        const bool has_fault_latched = payload_helper_functions::read_bool(payload_data, payload_length, 0U, state.fault_latched);
        const bool has_controller_time = payload_helper_functions::read_u32_le(payload_data, payload_length, 1U, state.controller_time_ms);

        if ((has_fault_latched == true) && (has_controller_time == true))
        {
            motion_mcu_incoming_state::set_safety_status(state);
        }
    }
}
