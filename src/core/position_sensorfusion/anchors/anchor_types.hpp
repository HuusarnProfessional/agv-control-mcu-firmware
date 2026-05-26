#pragma once

#include <cstdint>

namespace position_sensorfusion_anchors
{
    enum class anchor_type : std::uint8_t
    {
        none = 0U,
        heading_transform = 1U,
        position_only = 2U
    };

    struct global_reference
    {
        bool valid = false;
        bool has_heading = false;
        std::int64_t x_um = 0;
        std::int64_t y_um = 0;
        std::int64_t z_um = 0;
        std::int32_t heading_urad = 0;
        std::uint16_t confidence_position = 0U;
        std::uint16_t confidence_heading = 0U;
        std::uint32_t sample_id = 0U;
        std::uint32_t received_time_ms = 0U;
        bool has_local_reference = false;
        std::int64_t local_x_um = 0;
        std::int64_t local_y_um = 0;
        std::int32_t local_heading_urad = 0;
        std::uint16_t pose_id = 0U;
        std::uint8_t branch_id = 0U;
    };

    struct candidate
    {
        bool valid = false;
        anchor_type type = anchor_type::none;
        std::uint16_t confidence = 0U;
        global_reference reference = {};
    };

    struct current_reference
    {
        bool valid = false;
        bool has_heading = false;
        std::uint8_t branch_id = 0U;
        std::int64_t x_um = 0;
        std::int64_t y_um = 0;
        std::int64_t local_x_um = 0;
        std::int64_t local_y_um = 0;
        std::int32_t heading_urad = 0;
        std::int32_t rotation_urad = 0;
    };

    struct branch_request
    {
        bool valid = false;
        anchor_type type = anchor_type::none;
        std::uint16_t pose_id = 0U;
        std::uint8_t branch_id = 0U;
        std::uint16_t confidence = 0U;
    };

    struct reference_activation
    {
        bool valid = false;
        anchor_type type = anchor_type::none;
        bool is_initial_reference = false;
        bool is_mission_seed = false;
        bool has_saved_global_heading = false;
        std::uint16_t source_pose_id = 0U;
        std::uint8_t source_branch_id = 0U;
        std::uint32_t activation_time_ms = 0U;
        std::uint16_t confidence = 0U;
        std::int32_t saved_global_heading_urad = 0;
        global_reference reference = {};
    };
}
