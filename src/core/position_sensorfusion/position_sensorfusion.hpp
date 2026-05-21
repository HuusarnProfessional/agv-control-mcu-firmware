#pragma once

#include <cstdint>

namespace position_sensorfusion
{
    struct output_snapshot
    {
        bool has_pose = false;
        std::int64_t x_um = 0;
        std::int64_t y_um = 0;
        std::int32_t heading_urad = 0;
        std::uint16_t confidence_position = 0U;
        std::uint16_t confidence_heading = 0U;
        std::uint16_t pose_id = 0U;
        std::uint8_t branch_id = 0U;
    };

    void set_output(const output_snapshot &snapshot);

    output_snapshot read_output();
}
