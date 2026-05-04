#include "rotate_delta_payload.hpp"

#include "../../motion_mcu_routes.hpp"
#include "../../payload_helper_functions.hpp"
#include "../outgoing_payload_definition.hpp"

namespace rotate_delta_payload
{
    bool send(
        std::int32_t linear_velocity_mm_s,
        std::int32_t yaw_rate_mdeg_s,
        std::int64_t target_rotation_urad,
        bool has_rotation_drive_tuning,
        std::int32_t rotation_min_drive_u,
        std::int32_t rotation_startup_drive_u)
    {
        outgoing_payload_definition::payload_buffer payload = {};
        payload.payload_id = static_cast<std::uint8_t>(motion_mcu_routes::outgoing_payload_id::rotate_delta);
        payload.payload_length = 25U;

        if (payload_helper_functions::write_i32_le(payload.payload_data, sizeof(payload.payload_data), 0U, linear_velocity_mm_s) == false)
        {
            return false;
        }

        if (payload_helper_functions::write_i32_le(payload.payload_data, sizeof(payload.payload_data), 4U, yaw_rate_mdeg_s) == false)
        {
            return false;
        }

        if (payload_helper_functions::write_i64_le(payload.payload_data, sizeof(payload.payload_data), 8U, target_rotation_urad) == false)
        {
            return false;
        }

        if (payload_helper_functions::write_bool(payload.payload_data, sizeof(payload.payload_data), 16U, has_rotation_drive_tuning) == false)
        {
            return false;
        }

        if (payload_helper_functions::write_i32_le(payload.payload_data, sizeof(payload.payload_data), 17U, rotation_min_drive_u) == false)
        {
            return false;
        }

        if (payload_helper_functions::write_i32_le(payload.payload_data, sizeof(payload.payload_data), 21U, rotation_startup_drive_u) == false)
        {
            return false;
        }

        return outgoing_payload_definition::send_payload(payload);
    }
}
