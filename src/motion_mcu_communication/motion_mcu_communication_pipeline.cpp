#include "motion_mcu_communication_pipeline.hpp"

#include "incoming_payloads/incoming_motion_mcu_pipeline.hpp"
#include "outgoing_payloads/outgoing_motion_mcu_pipeline.hpp"

namespace motion_mcu_communication_pipeline
{
    void init()
    {
        incoming_motion_mcu_pipeline::init();
        outgoing_motion_mcu_pipeline::init();
    }

    void tick(std::uint32_t now_ms)
    {
        incoming_motion_mcu_pipeline::tick(now_ms);
        outgoing_motion_mcu_pipeline::tick(now_ms);
    }
}
