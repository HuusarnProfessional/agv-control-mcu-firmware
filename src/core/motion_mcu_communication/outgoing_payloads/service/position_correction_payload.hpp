#pragma once

#include <cstdint>

namespace position_correction_payload
{
    bool send(std::uint8_t pose_id, std::uint8_t branch_id);
}