#include "robot_control.hpp"

#include "../motion_mcu_communication/incoming_payloads/incoming_motion_mcu_pipeline.hpp"
#include "../motion_mcu_communication/outgoing_payloads/outgoing_motion_mcu_pipeline.hpp"
#include "../motion_mcu_communication/outgoing_payloads/service/obstacle_safety_control_payload.hpp"
#include "../global_positioning/global_positioning_pipeline.hpp"
#include "../position_sensorfusion/position_sensorfusion_pipeline.hpp"
#include "./primitives/command_speed/command_speed_state.hpp"
#include "./primitives/motion_primitive/motion_primitive_status_monitor.hpp"
#include "./primitives/pause/pause_pipeline.hpp"
#include "../mission/mission_pipeline.hpp"
#include "../pure_pursuit/pure_pursuit_pipeline.hpp"
#include "../bluetooth_communication/bluetooth_communication_pipeline.hpp"
#include "../bluetooth_communication/debug_capture/position_capture_test_pipeline.hpp"

namespace
{
    constexpr std::uint32_t obstacle_safety_default_retry_period_ms = 500U;

    bool obstacle_safety_default_sent = false;
    std::uint32_t next_obstacle_safety_default_retry_ms = 0U;

    void reset_default_outputs()
    {
        obstacle_safety_default_sent = false;
        next_obstacle_safety_default_retry_ms = 0U;
    }

    void send_default_outputs_if_needed(std::uint32_t now_ms)
    {
        if (obstacle_safety_default_sent == true)
        {
            return;
        }

        if (now_ms < next_obstacle_safety_default_retry_ms)
        {
            return;
        }

        if (obstacle_safety_control_payload::send(false) == true)
        {
            obstacle_safety_default_sent = true;
            return;
        }

        next_obstacle_safety_default_retry_ms = now_ms + obstacle_safety_default_retry_period_ms;
    }
}

namespace robot_control
{
    void init()
    {
        reset_default_outputs();
        incoming_motion_mcu_pipeline::init();
        outgoing_motion_mcu_pipeline::init();
        global_positioning_pipeline::init();
        position_sensorfusion_pipeline::init();
        command_speed_state::init();
        motion_primitive_status_monitor::init();
        pause_pipeline::init();
        mission_pipeline::init();
        pure_pursuit_pipeline::init();
        position_capture_test_pipeline::init();
        bluetooth_communication_pipeline::init();
    }

    void tick(std::uint32_t now_ms)
    {
        incoming_motion_mcu_pipeline::tick(now_ms);
        outgoing_motion_mcu_pipeline::tick(now_ms);
        send_default_outputs_if_needed(now_ms);
        global_positioning_pipeline::tick(now_ms);
        position_sensorfusion_pipeline::tick(now_ms);
        pause_pipeline::tick(now_ms);
        motion_primitive_status_monitor::tick(now_ms);
        mission_pipeline::tick(now_ms);
        pure_pursuit_pipeline::tick(now_ms);
        position_capture_test_pipeline::tick(now_ms);
        bluetooth_communication_pipeline::tick(now_ms);
    }
}
