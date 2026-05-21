#include "power_status_payload.hpp"

#include "../../payload_helper_functions.hpp"
#include "../../state/incoming/incoming_state.hpp"

namespace power_status_payload
{
    void handle(const std::uint8_t *payload_data, std::uint8_t payload_length)
    {
        motion_mcu_incoming_state::power_status_state state = {};
        const bool has_voltage = payload_helper_functions::read_u32_le(payload_data, payload_length, 0U, state.voltage_mv);
        const bool has_time = payload_helper_functions::read_u32_le(payload_data, payload_length, 4U, state.sample_time_ms);
        const bool has_status = payload_helper_functions::read_u8(payload_data, payload_length, 8U, state.status);

        if ((has_voltage == true) && (has_time == true) && (has_status == true))
        {
            motion_mcu_incoming_state::set_power_status(state);
        }
    }
}
