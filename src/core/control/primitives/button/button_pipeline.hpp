#pragma once

#include <cstdint>

namespace button_pipeline
{
    void init(std::uint8_t start_mission_button_pin, int active_level);

    void tick(std::uint32_t now_ms);
}
