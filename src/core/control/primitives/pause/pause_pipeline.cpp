#include "pause_pipeline.hpp"

#include <Arduino.h>

namespace
{
    bool g_is_active = false;
    std::uint32_t g_start_time_ms = 0U;
    std::uint32_t g_duration_ms = 0U;
}

namespace pause_pipeline
{
    void init()
    {
        g_is_active = false;
        g_start_time_ms = 0U;
        g_duration_ms = 0U;
    }

    bool request_pause(std::uint32_t duration_ms)
    {
        if (duration_ms == 0U)
        {
            g_is_active = false;
            g_start_time_ms = 0U;
            g_duration_ms = 0U;
            return true;
        }

        g_is_active = true;
        g_start_time_ms = millis();
        g_duration_ms = duration_ms;
        return true;
    }

    void tick(std::uint32_t now_ms)
    {
        if (g_is_active == false)
        {
            return;
        }

        const std::uint32_t elapsed_ms = now_ms - g_start_time_ms;

        if (elapsed_ms < g_duration_ms)
        {
            return;
        }

        g_is_active = false;
    }

    bool is_active()
    {
        return g_is_active;
    }
}
