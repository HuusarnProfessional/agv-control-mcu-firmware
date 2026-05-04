#pragma once

#include <cstdint>

namespace motion_mcu_heartbeat
{
    struct snapshot
    {
        bool has_seen_packet = false;
        bool packet_timed_out = false;
        std::uint32_t last_packet_time_ms = 0U;
    };

    void init();
    void notify_packet_received(std::uint32_t received_time_ms);
    void tick(std::uint32_t now_ms);
    snapshot read_snapshot();
}
