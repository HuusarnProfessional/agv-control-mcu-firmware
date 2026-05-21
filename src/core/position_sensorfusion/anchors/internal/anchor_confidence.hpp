#pragma once

#include <cstdint>

#include "../../filtered_global/filtered_global.hpp"

namespace anchor_confidence
{
    std::uint16_t median_position_confidence(const filtered_global::sample *samples, std::uint8_t sample_count);

    std::uint32_t median_residual_um(const std::uint32_t *residuals_um, std::uint8_t sample_count);
}
