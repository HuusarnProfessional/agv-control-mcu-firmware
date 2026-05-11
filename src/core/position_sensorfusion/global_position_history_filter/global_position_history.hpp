#pragma once

#include <cstdint>

namespace uwb_position_history
{
    constexpr std::uint8_t history_size = 8U;

    struct sample
    {
        bool valid = false;
        std::uint32_t sample_id = 0U;
        std::uint32_t received_time_ms = 0U;
        std::int64_t x_um = 0;
        std::int64_t y_um = 0;
        std::int64_t z_um = 0;
        std::uint16_t confidence_position = 0U;
    };

    struct history_state
    {
        sample samples[history_size] = {};
        std::uint8_t count = 0U;
    };

    void clear(history_state &state);

    void push(history_state &state, const sample &new_sample);

    bool has_last_sample(const history_state &state);

    sample get_last_sample(const history_state &state);

    bool has_two_samples(const history_state &state);

    sample get_previous_sample(const history_state &state);

    std::int64_t calculate_distance_um(const sample &left, const sample &right);

    std::uint32_t calculate_time_delta_ms(const sample &older_sample, const sample &newer_sample);
}