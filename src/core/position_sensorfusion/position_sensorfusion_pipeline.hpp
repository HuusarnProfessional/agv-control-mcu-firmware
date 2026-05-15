#pragma once

#include <cstdint>

namespace position_sensorfusion_pipeline
{
    void init();

    void set_bypass_offset_fusion(bool enabled);

    bool is_bypass_offset_fusion_enabled();

    void set_local_only_mode(bool enabled);

    bool is_local_only_mode_enabled();

    void tick(std::uint32_t now_ms);
}
