#pragma once

#include <cstdint>

namespace position_sensorfusion_pipeline
{
    void init();

    void tick(std::uint32_t now_ms);
}
