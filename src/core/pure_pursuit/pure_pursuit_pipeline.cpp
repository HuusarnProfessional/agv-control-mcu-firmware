#include "pure_pursuit_pipeline.hpp"

#include "pure_pursuit.hpp"
#include "pure_pursuit_tuning.hpp"
#include "../motion_mcu_communication/heartbeat/motion_mcu_heartbeat.hpp"
#include "../position_sensorfusion/position_sensorfusion.hpp"

namespace
{
    position_sensorfusion::output_snapshot latest_valid_position = {};
    std::uint32_t latest_valid_position_time_ms = 0U;
    bool has_latest_valid_position = false;

    position_sensorfusion::output_snapshot select_control_position(const position_sensorfusion::output_snapshot &current_position, std::uint32_t now_ms)
    {
        if (current_position.has_pose == true)
        {
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

    void tick(std::uint32_t now_ms)
    {
        const position_sensorfusion::output_snapshot current_position = position_sensorfusion::read_output();
        const position_sensorfusion::output_snapshot local_position = select_control_position(current_position, now_ms);
        const motion_mcu_heartbeat::snapshot heartbeat = motion_mcu_heartbeat::read_snapshot();
        pure_pursuit::tick(now_ms, local_position, heartbeat);
    }
}
