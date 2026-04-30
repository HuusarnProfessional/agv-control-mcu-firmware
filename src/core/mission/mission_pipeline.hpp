#pragma once

#include <cstdint>

namespace mission_pipeline
{
    void init();

    void tick(std::uint32_t now_ms);
}
