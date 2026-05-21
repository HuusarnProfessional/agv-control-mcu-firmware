#pragma once

#include <cstdint>

#include "../../motion_mcu_communication/state/incoming/incoming_state.hpp"
#include "filtered_global.hpp"

namespace filtered_global_pipeline
{
    void init();

    filtered_global::output_snapshot tick(std::uint32_t now_ms, const motion_mcu_incoming_state::local_position_state &local_position);
}
