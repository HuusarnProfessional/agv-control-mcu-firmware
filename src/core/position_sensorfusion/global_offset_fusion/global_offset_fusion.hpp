#pragma once

#include <cstdint>

#include "../local_position_alignment_to_global/local_position_alignment_to_global.hpp"
#include "../global_position_history_filter/global_position_history_filter.hpp"

namespace global_offset_fusion
{
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

        bool has_offset = false;
        bool has_heading_offset = false;
    };

    void init();

    output_snapshot update(const local_position_alignment_to_global::output_snapshot &local_position, const global_position_history_filter::output_snapshot &global_position);

    output_snapshot read_output();
}
