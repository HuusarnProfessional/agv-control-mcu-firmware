#pragma once

#include <cstdint>

namespace pure_pursuit_pipeline
{
    void init();

    void tick(std::uint32_t now_ms);
}
