#pragma once

#include <cstdint>

#include "../pure_pursuit.hpp"

namespace pure_pursuit_internal
{
    void send_motion_command(pure_pursuit::snapshot &snapshot, std::int32_t linear_velocity_mm_s, std::int32_t yaw_rate_mdeg_s);

    void send_stop_motion(pure_pursuit::snapshot &snapshot);
}
