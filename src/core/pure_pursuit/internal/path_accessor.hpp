#pragma once

#include <cstdint>

namespace pure_pursuit_internal
{
    struct path_point
    {
        std::int16_t x_mm = 0;
        std::int16_t y_mm = 0;
    };

    bool get_point_count(std::uint16_t part_number, std::uint16_t &point_count_out);

    bool get_point(std::uint16_t part_number, std::uint16_t point_index, path_point &point_out);
}
