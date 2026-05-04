#pragma once

#include <cstdint>

namespace command_speed_state
{
    void init();

    bool set_requested_speed_mm_s(std::uint16_t speed_mm_s);

    std::uint16_t get_requested_speed_mm_s();
}
