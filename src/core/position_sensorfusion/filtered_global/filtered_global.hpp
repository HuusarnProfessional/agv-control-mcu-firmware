#pragma once

#include <cstdint>

namespace filtered_global
{
    struct sample
    {
        bool valid = false;
        std::uint32_t sample_id = 0U;
        std::uint32_t request_id = 0U;
        std::uint32_t received_time_ms = 0U;
        std::int64_t x_um = 0;
        std::int64_t y_um = 0;
        std::int64_t z_um = 0;
        std::uint16_t raw_confidence_position = 0U;
        std::uint16_t confidence_position = 0U;
        bool has_local_reference = false;
        std::int64_t local_x_um = 0;
        std::int64_t local_y_um = 0;
        std::int32_t local_heading_urad = 0;
        std::uint16_t pose_id = 0U;
        std::uint8_t branch_id = 0U;
    };

    struct output_snapshot
    {
        bool has_position = false;
        std::int64_t x_um = 0;
        std::int64_t y_um = 0;
        std::int64_t z_um = 0;
        std::uint16_t confidence_position = 0U;
        bool is_new_sample = false;
        bool accepted = false;
        bool rejected = false;
        std::uint16_t raw_confidence_position = 0U;
        std::uint16_t history_confidence = 0U;
        std::uint32_t sample_id = 0U;
        std::uint32_t request_id = 0U;
        std::uint32_t received_time_ms = 0U;
    };

    void init();

    output_snapshot update(const sample &raw_sample, std::uint32_t now_ms);

    output_snapshot read_output(std::uint32_t now_ms);

    std::uint8_t history_count();

    bool read_history_sample(std::uint8_t index, sample &sample_out);
}
