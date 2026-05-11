#pragma once

#include <cstdint>

#include "../global_position_heading/global_position_heading.hpp"

namespace alignment_confidence
{
    std::uint16_t multiply(std::uint16_t left, std::uint16_t right);

    std::uint16_t calculate_anchor_score(const global_position_heading::output_snapshot &global_position, std::uint32_t now_ms);
}