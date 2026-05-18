#pragma once

#include <cstdint>

namespace pure_pursuit_tuning
{
    constexpr std::int16_t k_mission_linear_speed_mm_s = 180;
    constexpr std::int16_t k_mission_min_linear_speed_mm_s = 90;
    constexpr std::uint16_t k_mission_speed_scale_reference_mm_s = 200U;
    constexpr std::int32_t k_mission_turn_only_yaw_rate_mdeg_s = 70000;
    constexpr std::int32_t k_mission_max_yaw_rate_mdeg_s = 70000;
    constexpr double k_mission_heading_p_yaw_rate_mdeg_s_per_deg = 300.0;
    constexpr double k_mission_lookahead_mm = 230.0;
    constexpr double k_mission_goal_tolerance_mm = 120.0;
    constexpr double k_mission_speed_slowdown_heading_error_deg = 20.0;
    constexpr double k_mission_turn_only_heading_error_deg = 50.0;
    constexpr double k_mission_min_driveable_target_distance_mm = 120.0;
    constexpr double k_mission_min_driveable_forward_mm = 30.0;
    constexpr double k_mission_min_turn_radius_mm = 180.0;
    constexpr double k_mission_driveable_target_scan_start_mm = 100.0;
    constexpr double k_mission_driveable_target_scan_step_mm = 45.0;
    constexpr std::uint8_t k_mission_driveable_target_scan_count = 16u;
    constexpr std::uint16_t k_mission_max_chunk_iterations = 64u;
    constexpr std::uint16_t k_mission_max_closest_segment_checks = 48u;
    constexpr double k_mission_max_closest_progress_advance_mm = 700.0;
    constexpr std::uint16_t k_mission_max_lookahead_segment_steps = 48u;
    constexpr std::uint32_t k_mission_pose_hold_time_ms = 250U;
    constexpr std::uint32_t k_mission_pose_jump_guard_window_ms = 1000U;
    constexpr double k_mission_pose_jump_base_margin_mm = 250.0;
    constexpr double k_mission_pose_jump_speed_mm_s = 1500.0;
    constexpr double k_pi = 3.141592653589793;
}
