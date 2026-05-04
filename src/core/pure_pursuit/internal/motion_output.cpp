#include "motion_output.hpp"

#include "../../motion_mcu_communication/outgoing_payloads/runtime/motion_command_payload.hpp"

namespace pure_pursuit_internal
{
    void send_motion_command(pure_pursuit::snapshot &snapshot, std::int32_t linear_velocity_mm_s, std::int32_t yaw_rate_mdeg_s)
    {
        snapshot.linear_velocity_mm_s = linear_velocity_mm_s;
        snapshot.yaw_rate_mdeg_s = yaw_rate_mdeg_s;
        motion_command_payload::send(true, linear_velocity_mm_s, yaw_rate_mdeg_s);
    }

    void send_stop_motion(pure_pursuit::snapshot &snapshot)
    {
        snapshot.linear_velocity_mm_s = 0;
        snapshot.yaw_rate_mdeg_s = 0;
        motion_command_payload::send(false, 0, 0);
    }
}
