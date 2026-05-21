#pragma once

#include <cstdint>

#include "../../filtered_global/filtered_global.hpp"

namespace huber_pca_fit
{
    struct fit_result
    {
        bool valid = false;
        double direction_x = 1.0;
        double direction_y = 0.0;
        std::uint32_t residuals_um[15] = {};
        std::uint32_t median_residual_um = 0U;
    };

    fit_result fit(const filtered_global::sample *samples, std::uint8_t sample_count);
}
