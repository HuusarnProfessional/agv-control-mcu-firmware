#pragma once

#include <cstdint>

namespace motion_command_payload
{
    bool send(bool drive_enabled, std::int32_t linear_velocity_mm_s, std::int32_t yaw_rate_mdeg_s);
}
