#include "pure_pursuit_pipeline.hpp"

#include "pure_pursuit.hpp"
#include "../motion_mcu_communication/heartbeat/motion_mcu_heartbeat.hpp"
#include "../position_sensorfusion/position_sensorfusion.hpp"

namespace pure_pursuit_pipeline
{
    void init()
    {
        pure_pursuit::init();
    }

    void tick(std::uint32_t now_ms)
    {
        const position_sensorfusion::output_snapshot local_position = position_sensorfusion::read_output();
        const motion_mcu_heartbeat::snapshot heartbeat = motion_mcu_heartbeat::read_snapshot();
        pure_pursuit::tick(now_ms, local_position, heartbeat);
    }
}
