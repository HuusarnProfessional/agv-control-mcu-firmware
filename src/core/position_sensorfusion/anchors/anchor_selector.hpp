#pragma once

#include <cstdint>

#include "../../motion_mcu_communication/state/incoming/incoming_state.hpp"
#include "anchor_types.hpp"

namespace anchor_selector
{
    enum class decision_state : std::uint8_t
    {
        none = 0U,
        local_missing = 1U,
        pending = 2U,
        settling = 3U,
        missing_current_reference = 4U,
        rejected = 5U,
        requested = 6U,
        activated = 7U
    };

    enum class reject_reason : std::uint8_t
    {
        none = 0U,
        invalid_candidate = 1U,
        invalid_reference = 2U,
        missing_local_reference = 3U,
        missing_pose_id = 4U,
        confidence_low = 5U,
        missing_heading = 6U,
        outside_safe_area = 7U,
        pose_not_replayable = 8U,
        heading_inconsistent = 9U,
        jump_too_large = 10U,
        margin_not_met = 11U,
        duplicate_sample = 12U,
        request_interval = 13U,
        startup_delay = 14U
    };

    struct decision_snapshot
    {
        bool valid = false;
        decision_state state = decision_state::none;
        reject_reason reason = reject_reason::none;
        position_sensorfusion_anchors::anchor_type selected_type = position_sensorfusion_anchors::anchor_type::none;
        std::uint16_t required_confidence = 0U;
        std::uint16_t local_confidence = 0U;
        std::uint16_t heading_confidence = 0U;
        std::uint16_t position_confidence = 0U;
        std::uint16_t heading_pose_id = 0U;
        std::uint16_t position_pose_id = 0U;
        std::uint32_t heading_sample_id = 0U;
        std::uint32_t position_sample_id = 0U;
        bool position_projection_valid = false;
        std::int64_t position_reference_x_um = 0;
        std::int64_t position_reference_y_um = 0;
        std::int64_t position_projected_x_um = 0;
        std::int64_t position_projected_y_um = 0;
        std::int64_t position_jump_x_um = 0;
        std::int64_t position_jump_y_um = 0;
        std::int32_t position_projection_rotation_urad = 0;
    };

    struct output_snapshot
    {
        position_sensorfusion_anchors::branch_request request = {};
        position_sensorfusion_anchors::reference_activation activation = {};
        decision_snapshot decision = {};
        bool pending = false;
        bool settling = false;
    };

    void init();

    void reset_runtime_state();

    void set_position_jump_guard_enabled(bool enabled);

    bool is_position_jump_guard_enabled();

    output_snapshot update(const motion_mcu_incoming_state::local_position_state &local_position, const position_sensorfusion_anchors::current_reference &current_reference, const position_sensorfusion_anchors::candidate &heading_candidate, const position_sensorfusion_anchors::candidate &position_candidate, std::uint32_t now_ms);
}
