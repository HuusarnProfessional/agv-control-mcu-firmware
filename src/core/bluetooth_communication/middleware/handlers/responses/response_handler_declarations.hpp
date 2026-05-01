#pragma once

namespace response_handlers
{
    bool handle_pong();
    bool handle_ok();
    bool handle_fail();
    bool handle_speed();
    bool handle_position_local();
    bool handle_position_global();
    bool handle_mission_part_current();
    bool handle_armed();
}
