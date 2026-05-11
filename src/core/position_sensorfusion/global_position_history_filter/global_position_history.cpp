#include "uwb_position_history.hpp"

#include <cmath>
#include <cstdint>

namespace uwb_position_history
{
    void clear(history_state &state)
    {
        state = {};
    }

    void push(history_state &state, const sample &new_sample)
    {
        if (state.count > 0U)
        {
            for (std::uint8_t index = state.count; index > 0U; index--)
            {
                if (index >= history_size)
                {
                    continue;
                }

                state.samples[index] = state.samples[index - 1U];
            }
        }

        state.samples[0U] = new_sample;

        if (state.count < history_size)
        {
            state.count++;
        }
    }

    bool has_last_sample(const history_state &state)
    {
        if (state.count == 0U)
        {
            return false;
        }

        if (state.samples[0U].valid == false)
        {
            return false;
        }

        return true;
    }

    sample get_last_sample(const history_state &state)
    {
        if (has_last_sample(state) == false)
        {
            return {};
        }

        return state.samples[0U];
    }

    bool has_two_samples(const history_state &state)
    {
        if (state.count < 2U)
        {
            return false;
        }

        if (state.samples[0U].valid == false)
        {
            return false;
        }

        if (state.samples[1U].valid == false)
        {
            return false;
        }

        return true;
    }

    sample get_previous_sample(const history_state &state)
    {
        if (has_two_samples(state) == false)
        {
            return {};
        }

        return state.samples[1U];
    }

    std::int64_t calculate_distance_um(const sample &left, const sample &right)
    {
        const std::int64_t delta_x_um = right.x_um - left.x_um;
        const std::int64_t delta_y_um = right.y_um - left.y_um;
        const double delta_x = static_cast<double>(delta_x_um);
        const double delta_y = static_cast<double>(delta_y_um);
        const double distance = std::sqrt((delta_x * delta_x) + (delta_y * delta_y));

        return static_cast<std::int64_t>(distance);
    }

    std::uint32_t calculate_time_delta_ms(const sample &older_sample, const sample &newer_sample)
    {
        if (newer_sample.received_time_ms < older_sample.received_time_ms)
        {
            return 0U;
        }

        return newer_sample.received_time_ms - older_sample.received_time_ms;
    }
}