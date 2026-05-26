#include "pure_pursuit_pipeline.hpp"

#include "pure_pursuit.hpp"
#include "pure_pursuit_tuning.hpp"
#include "../motion_mcu_communication/heartbeat/motion_mcu_heartbeat.hpp"
#include "../position_sensorfusion/position_sensorfusion.hpp"

#include <cmath>

namespace
{
    position_sensorfusion::output_snapshot latest_valid_position = {};
    std::uint32_t latest_valid_position_time_ms = 0U;
    bool has_latest_valid_position = false;

    std::uint32_t get_elapsed_ms(std::uint32_t now_ms, std::uint32_t previous_time_ms)
    {
        if (now_ms < previous_time_ms)
        {
            return 0U;
        }

        return now_ms - previous_time_ms;
    }

    double calculate_distance_mm(const position_sensorfusion::output_snapshot &left_position, const position_sensorfusion::output_snapshot &right_position)
    {
        const double delta_x_mm = static_cast<double>(left_position.x_um - right_position.x_um) / 1000.0;
        const double delta_y_mm = static_cast<double>(left_position.y_um - right_position.y_um) / 1000.0;

        return std::sqrt((delta_x_mm * delta_x_mm) + (delta_y_mm * delta_y_mm));
    }

    bool position_jump_is_plausible(const position_sensorfusion::output_snapshot &current_position, std::uint32_t now_ms)
    {
        if (has_latest_valid_position == false)
        {
            return true;
        }

        if (latest_valid_position.has_pose == false)
        {
            return true;
        }

        if (current_position.branch_id != latest_valid_position.branch_id)
        {
            return true;
        }

        const std::uint32_t elapsed_ms = get_elapsed_ms(now_ms, latest_valid_position_time_ms);

        if (elapsed_ms > pure_pursuit_tuning::k_mission_pose_jump_guard_window_ms)
        {
            return true;
        }

        const double distance_mm = calculate_distance_mm(current_position, latest_valid_position);
        const double allowed_distance_mm = pure_pursuit_tuning::k_mission_pose_jump_base_margin_mm + (pure_pursuit_tuning::k_mission_pose_jump_speed_mm_s * static_cast<double>(elapsed_ms) / 1000.0);

        if (distance_mm > allowed_distance_mm)
        {
            return false;
        }

        return true;
    }

    position_sensorfusion::output_snapshot select_control_position(const position_sensorfusion::output_snapshot &current_position, std::uint32_t now_ms)
    {
        if (current_position.has_pose == true)
        {
            const bool jump_is_plausible = position_jump_is_plausible(current_position, now_ms);

            if (jump_is_plausible == false)
            {
                return latest_valid_position;
            }

            latest_valid_position = current_position;
            latest_valid_position_time_ms = now_ms;
            has_latest_valid_position = true;
            return current_position;
        }

        if (has_latest_valid_position == false)
        {
            return current_position;
        }

        if ((now_ms - latest_valid_position_time_ms) > pure_pursuit_tuning::k_mission_pose_hold_time_ms)
        {
            return current_position;
        }

        return latest_valid_position;
    }
}

namespace pure_pursuit_pipeline
{
    void init()
    {
        latest_valid_position = {};
        latest_valid_position_time_ms = 0U;
        has_latest_valid_position = false;
        pure_pursuit::init();
    }

    void notify_pose_reset()
    {
        latest_valid_position = {};
        latest_valid_position_time_ms = 0U;
        has_latest_valid_position = false;
    }

    void tick(std::uint32_t now_ms)
    {
        const position_sensorfusion::output_snapshot current_position = position_sensorfusion::read_output();
        const position_sensorfusion::output_snapshot local_position = select_control_position(current_position, now_ms);
        const motion_mcu_heartbeat::snapshot heartbeat = motion_mcu_heartbeat::read_snapshot();
        pure_pursuit::tick(now_ms, local_position, heartbeat);
    }
}
