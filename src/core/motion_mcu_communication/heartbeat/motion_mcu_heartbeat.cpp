#include "motion_mcu_heartbeat.hpp"

namespace
{
    constexpr std::uint32_t packet_timeout_ms = 350U;
    motion_mcu_heartbeat::snapshot g_snapshot = {};
}

namespace motion_mcu_heartbeat
{
    void init()
    {
        g_snapshot = {};
    }

    void notify_packet_received(std::uint32_t received_time_ms)
    {
        g_snapshot.has_seen_packet = true;
        g_snapshot.packet_timed_out = false;
        g_snapshot.last_packet_time_ms = received_time_ms;
    }

    void tick(std::uint32_t now_ms)
    {
        if (g_snapshot.has_seen_packet == false)
        {
            return;
        }

        if (now_ms < g_snapshot.last_packet_time_ms)
        {
            g_snapshot.packet_timed_out = true;
            return;
        }

        if ((now_ms - g_snapshot.last_packet_time_ms) > packet_timeout_ms)
        {
            g_snapshot.packet_timed_out = true;
            return;
        }

        g_snapshot.packet_timed_out = false;
    }

    snapshot read_snapshot()
    {
        return g_snapshot;
    }
}
