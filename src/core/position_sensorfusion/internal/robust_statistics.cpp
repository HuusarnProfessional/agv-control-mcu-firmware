#include "robust_statistics.hpp"

namespace position_sensorfusion_internal
{
    void sort_values(std::int64_t *values, std::uint8_t count)
    {
        for (std::uint8_t outer_index = 0U; outer_index < count; outer_index++)
        {
            for (std::uint8_t inner_index = static_cast<std::uint8_t>(outer_index + 1U); inner_index < count; inner_index++)
            {
                if (values[inner_index] < values[outer_index])
                {
                    const std::int64_t temporary = values[outer_index];
                    values[outer_index] = values[inner_index];
                    values[inner_index] = temporary;
                }
            }
        }
    }

    std::int64_t median_value(std::int64_t *values, std::uint8_t count)
    {
        if (count == 0U)
        {
            return 0;
        }

        sort_values(values, count);

        const std::uint8_t middle_index = count / 2U;

        if ((count % 2U) == 1U)
        {
            return values[middle_index];
        }

        const std::int64_t left_value = values[middle_index - 1U];
        const std::int64_t right_value = values[middle_index];

        return (left_value + right_value) / 2;
    }
}
