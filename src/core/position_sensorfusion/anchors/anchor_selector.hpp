#pragma once

#include <cstdint>

#include "../../motion_mcu_communication/state/incoming/incoming_state.hpp"
#include "anchor_types.hpp"

namespace anchor_selector
{
    struct output_snapshot
    {
        position_sensorfusion_anchors::branch_request request = {};
        position_sensorfusion_anchors::reference_activation activation = {};
        bool pending = false;
        bool settling = false;
    };

    void init();

    void reset_runtime_state();

    output_snapshot update(const motion_mcu_incoming_state::local_position_state &local_position, const position_sensorfusion_anchors::current_reference &current_reference, const position_sensorfusion_anchors::candidate &heading_candidate, const position_sensorfusion_anchors::candidate &position_candidate, std::uint32_t now_ms);
}
