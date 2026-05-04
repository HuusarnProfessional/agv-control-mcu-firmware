#pragma once

#include <cstdint>

namespace drive_forward_payload
{
    bool send(std::int32_t velocity_mm_s, std::int64_t target_distance_um);
}
