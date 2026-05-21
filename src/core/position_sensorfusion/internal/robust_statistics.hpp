#pragma once

#include <cstdint>

namespace position_sensorfusion_internal
{
    void sort_values(std::int64_t *values, std::uint8_t count);

    std::int64_t median_value(std::int64_t *values, std::uint8_t count);
}
