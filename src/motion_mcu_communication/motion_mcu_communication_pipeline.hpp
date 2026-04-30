#pragma once

#include <cstdint>

namespace motion_mcu_communication_pipeline
{
    void init();

    void tick(std::uint32_t now_ms);
}
