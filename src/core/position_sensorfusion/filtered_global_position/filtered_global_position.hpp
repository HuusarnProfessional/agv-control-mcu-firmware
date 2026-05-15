#pragma once

#include <cstdint>

namespace filtered_global_position
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

        bool is_new_sample = false;
        bool accepted = false;
        bool rejected = false;

        std::uint16_t raw_confidence_position = 0U;
        std::uint16_t history_confidence = 0U;
        std::uint8_t accepted_sample_count = 0U;
        std::uint8_t heading_sample_count = 0U;
        std::int64_t heading_distance_um = 0;

        std::uint32_t sample_id = 0U;
        std::uint32_t request_id = 0U;
        std::uint32_t received_time_ms = 0U;
    };

    void init();

    output_snapshot update(std::uint32_t now_ms);

    output_snapshot read_output(std::uint32_t now_ms);
}
