#pragma once

#include <cstdint>

namespace board_esp32_wroom32
{
    void init();
    void tick(std::uint32_t now_ms);
}
