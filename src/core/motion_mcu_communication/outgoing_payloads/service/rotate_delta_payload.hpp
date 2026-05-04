#pragma once

#include <cstdint>

namespace rotate_delta_payload
{
    bool send(
        std::int32_t linear_velocity_mm_s,
        std::int32_t yaw_rate_mdeg_s,
        std::int64_t target_rotation_urad,
        bool has_rotation_drive_tuning,
        std::int32_t rotation_min_drive_u,
        std::int32_t rotation_startup_drive_u);
}
