#pragma once

#include <cstdint>

#include "../filtered_global.hpp"
#include "../filtered_global_tuning.hpp"

namespace filtered_global_history
{
    struct state
    {
        filtered_global::sample samples[filtered_global_tuning::history_size] = {};
        std::uint8_t count = 0U;
    };

    void reset(state &history);

    void push(state &history, const filtered_global::sample &sample);

    bool read(const state &history, std::uint8_t index, filtered_global::sample &sample_out);

    bool newest(const state &history, filtered_global::sample &sample_out);
}
