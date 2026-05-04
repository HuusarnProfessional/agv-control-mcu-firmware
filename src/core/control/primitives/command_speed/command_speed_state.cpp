#include "command_speed_state.hpp"

namespace
{
    constexpr std::uint16_t default_requested_speed_mm_s = 200U;
    constexpr std::uint16_t max_requested_speed_mm_s = 32767U;

    std::uint16_t g_requested_speed_mm_s = default_requested_speed_mm_s;
}

namespace command_speed_state
{
    void init()
    {
        g_requested_speed_mm_s = default_requested_speed_mm_s;
    }

    bool set_requested_speed_mm_s(std::uint16_t speed_mm_s)
    {
        if (speed_mm_s == 0U)
        {
            return false;
        }

        if (speed_mm_s > max_requested_speed_mm_s)
        {
            return false;
        }

        g_requested_speed_mm_s = speed_mm_s;
        return true;
    }

    std::uint16_t get_requested_speed_mm_s()
    {
        return g_requested_speed_mm_s;
    }
}
