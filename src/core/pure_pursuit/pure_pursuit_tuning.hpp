#pragma once

#include <cstdint>

namespace pure_pursuit_tuning
{
    constexpr std::int16_t k_mission_linear_speed_mm_s = 165;
    constexpr std::int16_t k_mission_min_linear_speed_mm_s = 41;
    constexpr std::int32_t k_mission_turn_only_yaw_rate_mdeg_s = 32000;
    constexpr std::int32_t k_mission_max_yaw_rate_mdeg_s = 32000;
    constexpr double k_mission_lookahead_mm = 350.0;
    constexpr double k_mission_goal_tolerance_mm = 60.0;
    constexpr double k_mission_speed_slowdown_heading_error_deg = 35.0;
    constexpr double k_mission_turn_only_heading_error_deg = 110.0;
    constexpr double k_mission_min_driveable_target_distance_mm = 120.0;
    constexpr double k_mission_min_driveable_forward_mm = 40.0;
    constexpr double k_mission_min_turn_radius_mm = 250.0;
    constexpr double k_mission_driveable_target_scan_start_mm = 80.0;
    constexpr double k_mission_driveable_target_scan_step_mm = 40.0;
    constexpr std::uint8_t k_mission_driveable_target_scan_count = 16u;
    constexpr std::uint16_t k_mission_max_chunk_iterations = 64u;
    constexpr std::uint16_t k_mission_max_closest_segment_checks = 48u;
    constexpr std::uint16_t k_mission_max_lookahead_segment_steps = 48u;
    constexpr double k_pi = 3.141592653589793;
}
