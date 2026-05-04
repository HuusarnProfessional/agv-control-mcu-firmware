#pragma once

#include <cstdint>

namespace pause_pipeline
{
    void init();

    bool request_pause(std::uint32_t duration_ms);

    void tick(std::uint32_t now_ms);

    bool is_active();
}
