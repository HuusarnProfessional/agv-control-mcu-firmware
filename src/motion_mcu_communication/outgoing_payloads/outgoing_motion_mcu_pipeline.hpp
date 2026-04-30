#pragma once

#include <cstdint>

namespace outgoing_motion_mcu_pipeline
{
    void init();

    void tick(std::uint32_t now_ms);
}
