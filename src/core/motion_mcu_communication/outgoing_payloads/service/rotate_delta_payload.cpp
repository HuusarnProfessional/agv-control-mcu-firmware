#include "rotate_delta_payload.hpp"

#include <Arduino.h>

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
        Serial.print("stm rotate_delta linear_velocity_mm_s=");
        Serial.println(static_cast<long>(linear_velocity_mm_s));
        Serial.print("stm rotate_delta yaw_rate_mdeg_s=");
        Serial.println(static_cast<long>(yaw_rate_mdeg_s));
        Serial.print("stm rotate_delta target_rotation_urad=");
        Serial.println(static_cast<long long>(target_rotation_urad));
        Serial.print("stm rotate_delta has_rotation_drive_tuning=");
        Serial.println(has_rotation_drive_tuning ? 1 : 0);
        Serial.print("stm rotate_delta rotation_min_drive_u=");
        Serial.println(static_cast<long>(rotation_min_drive_u));
        Serial.print("stm rotate_delta rotation_startup_drive_u=");
        Serial.println(static_cast<long>(rotation_startup_drive_u));

        outgoing_payload_definition::payload_buffer payload = {};
        payload.payload_id = static_cast<std::uint8_t>(motion_mcu_routes::outgoing_payload_id::rotate_delta);
        payload.payload_length = 25U;

        if (payload_helper_functions::write_i32_le(payload.payload_data, sizeof(payload.payload_data), 0U, linear_velocity_mm_s) == false)
        {
            Serial.println("stm rotate_delta write linear fail");
            return false;
        }

        if (payload_helper_functions::write_i32_le(payload.payload_data, sizeof(payload.payload_data), 4U, yaw_rate_mdeg_s) == false)
        {
            Serial.println("stm rotate_delta write yaw fail");
            return false;
        }

        if (payload_helper_functions::write_i64_le(payload.payload_data, sizeof(payload.payload_data), 8U, target_rotation_urad) == false)
        {
            Serial.println("stm rotate_delta write target fail");
            return false;
        }

        if (payload_helper_functions::write_bool(payload.payload_data, sizeof(payload.payload_data), 16U, has_rotation_drive_tuning) == false)
        {
            Serial.println("stm rotate_delta write tuning flag fail");
            return false;
        }

        if (payload_helper_functions::write_i32_le(payload.payload_data, sizeof(payload.payload_data), 17U, rotation_min_drive_u) == false)
        {
            Serial.println("stm rotate_delta write min_u fail");
            return false;
        }

        if (payload_helper_functions::write_i32_le(payload.payload_data, sizeof(payload.payload_data), 21U, rotation_startup_drive_u) == false)
        {
            Serial.println("stm rotate_delta write startup_u fail");
            return false;
        }

        const bool sent_ok = outgoing_payload_definition::send_payload(payload);

        if (sent_ok == true)
        {
            Serial.println("stm rotate_delta uart send ok");
        }
        else
        {
            Serial.println("stm rotate_delta uart send fail");
        }

        return sent_ok;
    }
}
