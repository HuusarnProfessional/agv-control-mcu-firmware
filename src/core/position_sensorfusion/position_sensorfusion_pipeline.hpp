#pragma once

#include <cstdint>

namespace position_sensorfusion_pipeline
{
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

    void start_entry_seeded_drive_forward_test();

    void tick(std::uint32_t now_ms);
}
