#pragma once

#include <cstdint>

#include "../../motion_mcu_communication/state/incoming/incoming_state.hpp"
#include "../anchors/anchor_types.hpp"

namespace local_to_global_transform
{
    struct output_snapshot
    {
        bool has_pose = false;
        bool has_transform = false;
        bool branch_matches = false;
        bool transform_activated = false;
        std::int64_t x_um = 0;
        std::int64_t y_um = 0;
        std::int32_t heading_urad = 0;
        std::int32_t rotation_urad = 0;
        std::uint16_t confidence_position = 0U;
        std::uint16_t confidence_heading = 0U;
        std::uint16_t pose_id = 0U;
        std::uint8_t branch_id = 0U;
        std::uint32_t reference_sample_id = 0U;
        bool is_mission_seed = false;
    };

    void init();

    void reset_runtime_state();

    output_snapshot update(const motion_mcu_incoming_state::local_position_state &local_position, const position_sensorfusion_anchors::reference_activation &activation);

    output_snapshot read_output(const motion_mcu_incoming_state::local_position_state &local_position);
}
