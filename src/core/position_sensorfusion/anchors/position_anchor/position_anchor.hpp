#pragma once

#include <cstdint>

#include "../anchor_types.hpp"

namespace position_anchor
{
    void init();

    position_sensorfusion_anchors::candidate update(bool has_reference_rotation, std::int32_t reference_rotation_urad);
}
