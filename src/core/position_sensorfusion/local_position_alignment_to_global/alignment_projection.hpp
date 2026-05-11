#pragma once

#include "../../motion_mcu_communication/state/incoming/incoming_state.hpp"

#include "alignment_anchor.hpp"
#include "local_position_alignment_to_global.hpp"

namespace alignment_projection
{
    local_position_alignment_to_global::output_snapshot project(const alignment_anchor::anchor_state &anchor, const motion_mcu_incoming_state::local_position_state &local_position);
}