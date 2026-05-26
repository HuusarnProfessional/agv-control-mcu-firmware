#pragma once

#include <cstddef>
#include <cstdint>

#include "../motion_mcu_communication/state/incoming/incoming_state.hpp"
#include "../position_sensorfusion/anchors/anchor_types.hpp"
#include "../position_sensorfusion/filtered_global/filtered_global.hpp"
#include "../position_sensorfusion/position_sensorfusion.hpp"

namespace position_trace_logger
{
    enum class event_action : std::uint8_t
    {
        none = 0U,
        request = 1U,
        activation = 2U,
        seed = 3U
    };

    struct anchor_event_snapshot
    {
        bool valid = false;
        event_action action = event_action::none;
        position_sensorfusion_anchors::anchor_type anchor_type = position_sensorfusion_anchors::anchor_type::none;
        std::uint16_t pose_id = 0U;
        std::uint8_t branch_id = 0U;
        std::uint16_t confidence = 0U;
        std::int64_t x_um = 0;
        std::int64_t y_um = 0;
        std::int32_t saved_global_heading_urad = 0;
        std::int32_t activation_local_heading_urad = 0;
    };

    struct anchor_decision_snapshot
    {
        bool valid = false;
        std::uint8_t selector_state = 0U;
        std::uint8_t selected_type = 0U;
        std::uint8_t reject_reason = 0U;
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

    void init();
    void clear();
    void set_enabled(bool enabled);
    bool is_enabled();
    void set_period_ms(std::uint16_t period_ms);
    std::uint16_t get_period_ms();
    std::uint16_t stored_count();
    std::uint32_t dropped_count();

    void tick(std::uint32_t now_ms,
              const motion_mcu_incoming_state::local_position_state &local_position,
              const filtered_global::output_snapshot &filtered_position,
              const position_sensorfusion::output_snapshot &sensorfusion_position,
              std::int32_t transform_rotation_urad,
              const anchor_event_snapshot &anchor_event,
              const anchor_decision_snapshot &anchor_decision);

    bool format_status(char *buffer_out, std::size_t capacity);
    bool format_packet(char *buffer_out, std::size_t capacity);
}
