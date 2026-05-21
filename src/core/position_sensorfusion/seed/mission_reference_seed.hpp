#pragma once

#include <cstdint>

#include "../../motion_mcu_communication/state/incoming/incoming_state.hpp"
#include "../anchors/anchor_types.hpp"

namespace mission_reference_seed
{
    void init();

    void reset_runtime_state();

    position_sensorfusion_anchors::reference_activation update(const motion_mcu_incoming_state::local_position_state &local_position, std::uint32_t now_ms);
}
