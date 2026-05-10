#pragma once

#include <cstdint>

namespace global_positioning_pipeline
{

void init();

void tick(std::uint32_t now_ms);

}
