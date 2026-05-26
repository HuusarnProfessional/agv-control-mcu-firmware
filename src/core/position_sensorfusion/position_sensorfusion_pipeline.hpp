#pragma once

#include <cstdint>

namespace position_sensorfusion_pipeline
{
    struct stop_relocalize_status_snapshot
    {
        bool enabled = false;
        bool active = false;
        bool external_hold = false;
        std::uint8_t phase = 0U;
        std::uint16_t local_confidence_position = 0U;
        std::uint16_t filtered_confidence_position = 0U;
        bool filtered_ready = false;
        std::uint32_t filtered_age_ms = 0U;
        std::uint32_t attempt_elapsed_ms = 0U;
        std::uint32_t phase_elapsed_ms = 0U;
        std::uint32_t phase_required_ms = 0U;
        std::uint32_t cooldown_remaining_ms = 0U;
        bool request_pending = false;
        std::uint16_t request_pose_id = 0U;
        std::uint8_t request_branch_id = 0U;
        std::uint8_t collected_sample_count = 0U;
        bool last_activation_performed = false;
        std::uint16_t last_activation_confidence = 0U;
        std::uint32_t last_activation_time_ms = 0U;
        std::uint8_t last_cancel_reason = 0U;
    };

    void init();

    void set_local_only_mode(bool enabled);

    bool is_local_only_mode_enabled();

    void set_heading_anchor_enabled(bool enabled);

    bool is_heading_anchor_enabled();

    void set_position_anchor_enabled(bool enabled);

    bool is_position_anchor_enabled();

    void set_position_anchor_direct_filtered_sample_mode(bool enabled);

    bool is_position_anchor_direct_filtered_sample_mode_enabled();

    void set_position_anchor_jump_guard_enabled(bool enabled);

    bool is_position_anchor_jump_guard_enabled();

    void set_stop_relocalize_test_enabled(bool enabled);

    bool is_stop_relocalize_test_enabled();

    stop_relocalize_status_snapshot read_stop_relocalize_status(std::uint32_t now_ms);

    void start_entry_seeded_drive_forward_test();

    void tick(std::uint32_t now_ms);
}
