#include "history_buffer.hpp"

namespace filtered_global_history
{
    void reset(state &history)
    {
        history = {};
    }

    void push(state &history, const filtered_global::sample &sample)
    {
        const std::uint8_t last_index = filtered_global_tuning::history_size - 1U;

        for (std::uint8_t index = last_index; index > 0U; index--)
        {
            history.samples[index] = history.samples[index - 1U];
        }

        history.samples[0U] = sample;

        if (history.count < filtered_global_tuning::history_size)
        {
            history.count++;
        }
    }

    bool read(const state &history, std::uint8_t index, filtered_global::sample &sample_out)
    {
        if (index >= history.count)
        {
            return false;
        }

        sample_out = history.samples[index];
        return sample_out.valid;
    }

    bool newest(const state &history, filtered_global::sample &sample_out)
    {
        return read(history, 0U, sample_out);
    }
}
