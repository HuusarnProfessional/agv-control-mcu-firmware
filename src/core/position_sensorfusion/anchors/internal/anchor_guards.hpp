#pragma once

#include "../../../motion_mcu_communication/state/incoming/incoming_state.hpp"
#include "../anchor_types.hpp"

namespace anchor_guards
{
    struct projected_candidate_snapshot
    {
        bool valid = false;
        std::int64_t projected_x_um = 0;
        std::int64_t projected_y_um = 0;
        std::int64_t jump_x_um = 0;
        std::int64_t jump_y_um = 0;
        std::int32_t rotation_urad = 0;
    };

    bool reference_is_inside_safe_area(const position_sensorfusion_anchors::candidate &candidate);

    bool candidate_pose_can_be_replayed(const motion_mcu_incoming_state::local_position_state &local_position, const position_sensorfusion_anchors::candidate &candidate);

    bool candidate_position_jump_is_safe(const motion_mcu_incoming_state::local_position_state &local_position, const position_sensorfusion_anchors::current_reference &current_reference, const position_sensorfusion_anchors::candidate &candidate);

    bool heading_is_consistent(const position_sensorfusion_anchors::current_reference &current_reference, const position_sensorfusion_anchors::candidate &candidate);

    projected_candidate_snapshot build_projected_candidate_snapshot(const motion_mcu_incoming_state::local_position_state &local_position, const position_sensorfusion_anchors::current_reference &current_reference, const position_sensorfusion_anchors::candidate &candidate);
}
