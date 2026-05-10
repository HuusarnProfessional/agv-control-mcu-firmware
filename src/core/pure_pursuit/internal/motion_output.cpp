#include "motion_output.hpp"

#include <Arduino.h>

#include "../../motion_mcu_communication/outgoing_payloads/runtime/motion_command_payload.hpp"

namespace pure_pursuit_internal
{
    void send_motion_command(pure_pursuit::snapshot &snapshot, std::int32_t linear_velocity_mm_s, std::int32_t yaw_rate_mdeg_s)
    {
        snapshot.linear_velocity_mm_s = linear_velocity_mm_s;
        snapshot.yaw_rate_mdeg_s = yaw_rate_mdeg_s;
        Serial.print("pure_pursuit cmd linear_velocity_mm_s=");
        Serial.println(static_cast<long>(linear_velocity_mm_s));
        Serial.print("pure_pursuit cmd yaw_rate_mdeg_s=");
        Serial.println(static_cast<long>(yaw_rate_mdeg_s));
        const bool send_ok = motion_command_payload::send(true, linear_velocity_mm_s, yaw_rate_mdeg_s);

        if (send_ok == true)
        {
            Serial.println("pure_pursuit send_motion_command pass");
        }
        else
        {
            Serial.println("pure_pursuit send_motion_command fail");
        }
    }

    void send_stop_motion(pure_pursuit::snapshot &snapshot)
    {
        snapshot.linear_velocity_mm_s = 0;
        snapshot.yaw_rate_mdeg_s = 0;
        Serial.println("pure_pursuit cmd stop");
        const bool send_ok = motion_command_payload::send(false, 0, 0);

        if (send_ok == true)
        {
            Serial.println("pure_pursuit send_stop_motion pass");
        }
        else
        {
            Serial.println("pure_pursuit send_stop_motion fail");
        }
    }
}
