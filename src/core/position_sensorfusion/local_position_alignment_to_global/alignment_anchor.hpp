#pragma once

#include <cstdint>

#include "../../motion_mcu_communication/state/incoming/incoming_state.hpp"
#include "../global_position_heading/global_position_heading.hpp"

namespace alignment_anchor
{
    struct anchor_state
    {
        bool valid = false;
        bool has_heading = false;

        std::int64_t global_x_um = 0;
        std::int64_t global_y_um = 0;
        std::int32_t global_heading_urad = 0;

        std::int64_t local_x_um = 0;
        std::int64_t local_y_um = 0;
        std::int32_t local_heading_urad = 0;

        std::int32_t heading_offset_urad = 0;

        std::uint16_t confidence_position = 0U;
        std::uint16_t confidence_heading = 0U;

        std::uint8_t local_branch_id = 0U;
        std::uint32_t global_sample_id = 0U;
    };

    void clear(anchor_state &anchor);

    void set_local_branch_id(anchor_state &anchor, std::uint8_t branch_id);

    anchor_state create_initial(const motion_mcu_incoming_state::local_position_state &local_position, const global_position_heading::output_snapshot &global_position, std::uint16_t anchor_score);

    anchor_state create_branch(const global_position_heading::output_snapshot &global_position, std::uint16_t anchor_score);
}