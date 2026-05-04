#include "outgoing_motion_mcu_pipeline.hpp"

#include "service/heartbeat_payload.hpp"

namespace
{
    constexpr std::uint32_t heartbeat_period_ms = 100U;
    std::uint32_t g_next_heartbeat_time_ms = 0U;
    bool g_heartbeat_initialized = false;
}

namespace outgoing_motion_mcu_pipeline
{
    void init()
    {
        g_next_heartbeat_time_ms = 0U;
        g_heartbeat_initialized = false;
    }

    void tick(std::uint32_t now_ms)
    {
        if (g_heartbeat_initialized == false)
        {
            g_next_heartbeat_time_ms = now_ms;
            g_heartbeat_initialized = true;
        }

        if (now_ms < g_next_heartbeat_time_ms)
        {
            return;
        }

        if (heartbeat_payload::send() == true)
        {
            g_next_heartbeat_time_ms = now_ms + heartbeat_period_ms;
        }
    }
}
