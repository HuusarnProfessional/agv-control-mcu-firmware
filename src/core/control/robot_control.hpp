#pragma once

#include <cstdint>

namespace robot_control
{
    void init();

    void tick(std::uint32_t now_ms);
}
