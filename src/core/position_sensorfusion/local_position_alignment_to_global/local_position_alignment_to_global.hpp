#pragma once

#include <cstdint>

#include "../../motion_mcu_communication/state/incoming/incoming_state.hpp"
#include "../global_position_heading/global_position_heading.hpp"

namespace local_position_alignment_to_global
{
    struct branch_request
    {
        bool has_request = false;
        std::uint8_t pose_id = 0U;
        std::uint8_t branch_id = 0U;
        std::uint16_t anchor_score = 0U;
        bool is_upgrade = false;
    };

    struct output_snapshot
    {
        bool has_pose = false;
        std::int64_t x_um = 0;
        std::int64_t y_um = 0;
        std::int32_t heading_urad = 0;
        std::uint16_t confidence_position = 0U;
        std::uint16_t confidence_heading = 0U;
        std::uint8_t pose_id = 0U;
        std::uint8_t branch_id = 0U;

        bool has_global_anchor = false;
        bool has_heading_anchor = false;

        branch_request request = {};
    };

    void init();

    output_snapshot update(const motion_mcu_incoming_state::local_position_state &local_position, const global_position_heading::output_snapshot &global_position, std::uint32_t now_ms);

    output_snapshot read_output(const motion_mcu_incoming_state::local_position_state &local_position);
}