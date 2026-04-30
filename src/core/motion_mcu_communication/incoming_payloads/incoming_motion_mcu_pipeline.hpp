#pragma once

#include <cstdint>

namespace incoming_motion_mcu_pipeline
{
    void init();

    void tick(std::uint32_t now_ms);
}
