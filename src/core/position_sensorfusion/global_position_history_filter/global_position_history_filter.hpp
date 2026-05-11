#pragma once

#include <cstdint>

#include "../global_position_heading/global_position_heading.hpp"

namespace uwb_position_history_filter
{
    struct output_snapshot
    {
        bool has_position = false;
        std::int64_t x_um = 0;
        std::int64_t y_um = 0;
        std::int64_t z_um = 0;

        std::uint16_t confidence_position = 0U;
        std::uint16_t original_confidence_position = 0U;
        std::uint16_t history_confidence = 0U;

        bool has_heading = false;
        std::int32_t heading_urad = 0;
        std::uint16_t confidence_heading = 0U;

        bool is_new_sample = false;
        bool rejected = false;

        std::uint32_t sample_id = 0U;
        std::uint32_t received_time_ms = 0U;
    };

    void init();

    output_snapshot update(const global_position_heading::output_snapshot &global_position);

    output_snapshot read_output();
}