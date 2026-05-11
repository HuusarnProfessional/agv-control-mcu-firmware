#pragma once

#include <cstdint>

#include "../../global_positioning/global_position_api.hpp"

namespace global_position_heading
{
    struct output_snapshot
    {
        bool has_position = false;
        std::int64_t x_um = 0;
        std::int64_t y_um = 0;
        std::int64_t z_um = 0;
        std::uint16_t confidence_position = 0U;

        bool has_heading = false;
        std::int32_t heading_urad = 0;
        std::uint16_t confidence_heading = 0U;

        std::uint32_t sample_id = 0U;
        std::uint32_t received_time_ms = 0U;
    };

    void init();

    output_snapshot update(const global_position_api::global_position_sample &sample);
}
