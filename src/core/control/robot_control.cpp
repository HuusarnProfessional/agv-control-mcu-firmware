#include "robot_control.hpp"

#include "../motion_mcu_communication/incoming_payloads/incoming_motion_mcu_pipeline.hpp"
#include "../motion_mcu_communication/outgoing_payloads/outgoing_motion_mcu_pipeline.hpp"
#include "../global_positioning/global_positioning_pipeline.hpp"
#include "../position_sensorfusion/position_sensorfusion_pipeline.hpp"
#include "./primitives/command_speed/command_speed_state.hpp"
#include "./primitives/motion_primitive/motion_primitive_status_monitor.hpp"
#include "./primitives/pause/pause_pipeline.hpp"
#include "../mission/mission_pipeline.hpp"
#include "../pure_pursuit/pure_pursuit_pipeline.hpp"
#include "../bluetooth_communication/bluetooth_communication_pipeline.hpp"
#include "../bluetooth_communication/debug_capture/position_capture_test_pipeline.hpp"

namespace robot_control
{
    void init()
    {
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
