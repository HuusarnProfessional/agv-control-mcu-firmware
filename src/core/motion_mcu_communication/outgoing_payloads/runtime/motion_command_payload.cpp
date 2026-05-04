#include "motion_command_payload.hpp"

#include "../../motion_mcu_routes.hpp"
#include "../../payload_helper_functions.hpp"
#include "../outgoing_payload_definition.hpp"

namespace motion_command_payload
{
    bool send(bool drive_enabled, std::int32_t linear_velocity_mm_s, std::int32_t yaw_rate_mdeg_s)
    {
        outgoing_payload_definition::payload_buffer payload = {};
        payload.payload_id = static_cast<std::uint8_t>(motion_mcu_routes::outgoing_payload_id::motion_command);
        payload.payload_length = 9U;

        if (payload_helper_functions::write_bool(payload.payload_data, sizeof(payload.payload_data), 0U, drive_enabled) == false)
        {
            return false;
        }

        if (payload_helper_functions::write_i32_le(payload.payload_data, sizeof(payload.payload_data), 1U, linear_velocity_mm_s) == false)
        {
            return false;
        }

        if (payload_helper_functions::write_i32_le(payload.payload_data, sizeof(payload.payload_data), 5U, yaw_rate_mdeg_s) == false)
        {
            return false;
        }

        return outgoing_payload_definition::send_payload(payload);
    }
}
