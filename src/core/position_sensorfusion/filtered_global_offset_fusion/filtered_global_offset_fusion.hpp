#pragma once

#include <cstdint>

#include "../filtered_global_position/filtered_global_position.hpp"
#include "../local_to_global_transform/local_to_global_transform.hpp"

namespace filtered_global_offset_fusion
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

        bool has_offset = false;
        bool has_heading_offset = false;
    };

    void init();

    void reset_runtime_state();

    output_snapshot update(const local_to_global_transform::output_snapshot &transformed_local_position, const filtered_global_position::output_snapshot &global_position);

    output_snapshot read_output();
}
