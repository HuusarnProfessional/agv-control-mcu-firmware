#include "geometry_helpers.hpp"

#include <cmath>

namespace position_sensorfusion_internal
{
    std::uint32_t elapsed_ms(std::uint32_t newer_time_ms, std::uint32_t older_time_ms)
    {
        if (newer_time_ms < older_time_ms)
        {
            return 0U;
        }

        return newer_time_ms - older_time_ms;
    }

    std::uint32_t absolute_time_difference_ms(std::uint32_t left_time_ms, std::uint32_t right_time_ms)
    {
        if (left_time_ms >= right_time_ms)
        {
            return left_time_ms - right_time_ms;
        }

        return right_time_ms - left_time_ms;
    }

    std::int64_t absolute_i64(std::int64_t value)
    {
        if (value < 0)
        {
            return -value;
        }

        return value;
    }

    std::int64_t calculate_distance_um(std::int64_t delta_x_um, std::int64_t delta_y_um)
    {
        const double delta_x = static_cast<double>(delta_x_um);
        const double delta_y = static_cast<double>(delta_y_um);
        const double distance = std::sqrt((delta_x * delta_x) + (delta_y * delta_y));

        return static_cast<std::int64_t>(std::llround(distance));
    }

    std::int32_t normalize_angle_urad(std::int32_t angle_urad)
    {
        const double normalized_double = std::remainder(static_cast<double>(angle_urad), static_cast<double>(two_pi_urad));
        std::int32_t normalized = static_cast<std::int32_t>(std::llround(normalized_double));

        if (normalized > pi_urad)
        {
            normalized -= two_pi_urad;
        }

        if (normalized < -pi_urad)
        {
            normalized += two_pi_urad;
        }

        return normalized;
    }

    std::int32_t absolute_angle_delta_urad(std::int32_t left_urad, std::int32_t right_urad)
    {
        std::int32_t delta_urad = normalize_angle_urad(left_urad - right_urad);

        if (delta_urad < 0)
        {
            delta_urad = -delta_urad;
        }

        return delta_urad;
    }

    void rotate_xy_um(std::int64_t x_um, std::int64_t y_um, std::int32_t rotation_urad, std::int64_t &rotated_x_um_out, std::int64_t &rotated_y_um_out)
    {
        const double rotation_rad = static_cast<double>(rotation_urad) / 1000000.0;
        const double cos_rotation = std::cos(rotation_rad);
        const double sin_rotation = std::sin(rotation_rad);
        const double rotated_x_um = (static_cast<double>(x_um) * cos_rotation) - (static_cast<double>(y_um) * sin_rotation);
        const double rotated_y_um = (static_cast<double>(x_um) * sin_rotation) + (static_cast<double>(y_um) * cos_rotation);

        rotated_x_um_out = static_cast<std::int64_t>(std::llround(rotated_x_um));
        rotated_y_um_out = static_cast<std::int64_t>(std::llround(rotated_y_um));
    }
}
