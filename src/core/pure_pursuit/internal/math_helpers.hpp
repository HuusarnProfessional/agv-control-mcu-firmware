#pragma once

#include <cstdint>

namespace pure_pursuit_internal
{
    double clamp_unit(double value);

    double normalize_angle_deg(double angle_deg);

    std::int32_t get_mission_linear_speed_mm_s();

    std::int32_t get_mission_min_linear_speed_mm_s();

    void apply_heading_speed_slowdown(std::int32_t &linear_velocity_mm_s, double abs_heading_error_deg);

    std::int32_t compute_tracking_yaw_rate_mdeg_s(double curvature, std::int32_t linear_velocity_mm_s);

    void apply_heading_p_yaw_correction(std::int32_t &yaw_rate_mdeg_s, double heading_error_deg);

    void apply_yaw_rate_speed_limit(std::int32_t &linear_velocity_mm_s, std::int32_t yaw_rate_mdeg_s);

    std::int32_t compute_turn_only_yaw_rate_mdeg_s(double forward_mm, double left_mm, double heading_error_deg);
}
