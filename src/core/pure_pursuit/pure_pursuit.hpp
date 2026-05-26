#pragma once

#include <cstdint>

#include "../motion_mcu_communication/heartbeat/motion_mcu_heartbeat.hpp"
#include "../position_sensorfusion/position_sensorfusion.hpp"

namespace pure_pursuit
{
    struct snapshot
    {
        bool active = false;
        bool complete = false;
        bool success = false;
        bool has_path = false;
        bool target_valid = false;
        bool target_forward_ok = false;
        bool target_curvature_ok = false;
        std::uint16_t part_number = 0u;
        std::uint16_t point_count = 0u;
        std::uint16_t closest_point_index = 0u;
        std::uint16_t lookahead_point_index = 0u;
        std::int32_t linear_velocity_mm_s = 0;
        std::int32_t yaw_rate_mdeg_s = 0;
        std::uint8_t stop_reason = 0U;
        double robot_x_mm = 0.0;
        double robot_y_mm = 0.0;
        double robot_heading_deg = 0.0;
        std::uint16_t robot_pose_id = 0U;
        std::uint8_t robot_branch_id = 0U;
        double target_x_mm = 0.0;
        double target_y_mm = 0.0;
        double target_curvature = 0.0;
        double target_distance_mm = 0.0;
        double forward_mm = 0.0;
        double left_mm = 0.0;
        double heading_error_deg = 0.0;
    };

    void init();

    bool start_part(std::uint16_t part_number);

    void set_external_hold(bool enabled);

    bool is_external_hold_enabled();

    void stop();

    void tick(std::uint32_t now_ms, const position_sensorfusion::output_snapshot &local_position, const motion_mcu_heartbeat::snapshot &heartbeat);

    snapshot read_snapshot();
}
