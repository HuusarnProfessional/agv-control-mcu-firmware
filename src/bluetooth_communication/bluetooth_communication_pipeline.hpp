#pragma once

#include <cstdint>

namespace bluetooth_communication_pipeline
{
    void init();

    void tick(std::uint32_t now_ms);
}
