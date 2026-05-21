#pragma once

#include <cstdint>

namespace position_sensorfusion_internal
{
    constexpr std::int32_t pi_urad = 3141593;
    constexpr std::int32_t two_pi_urad = 6283185;

    std::uint32_t elapsed_ms(std::uint32_t newer_time_ms, std::uint32_t older_time_ms);

    std::uint32_t absolute_time_difference_ms(std::uint32_t left_time_ms, std::uint32_t right_time_ms);

    std::int64_t absolute_i64(std::int64_t value);

    std::int64_t calculate_distance_um(std::int64_t delta_x_um, std::int64_t delta_y_um);

    std::int32_t normalize_angle_urad(std::int32_t angle_urad);

    std::int32_t absolute_angle_delta_urad(std::int32_t left_urad, std::int32_t right_urad);

    void rotate_xy_um(std::int64_t x_um, std::int64_t y_um, std::int32_t rotation_urad, std::int64_t &rotated_x_um_out, std::int64_t &rotated_y_um_out);
}
