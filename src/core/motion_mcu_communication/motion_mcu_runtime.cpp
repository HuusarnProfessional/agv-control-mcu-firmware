#include "motion_mcu_runtime.hpp"

namespace
{
    motion_mcu_runtime::local_position_state g_local_position = {};
    motion_mcu_runtime::safety_status_state g_safety_status = {};
    motion_mcu_runtime::power_status_state g_power_status = {};
}

namespace motion_mcu_runtime
{
    void init()
    {
        g_local_position = {};
        g_safety_status = {};
        g_power_status = {};
    }

    void set_local_position(const local_position_state &state)
    {
        g_local_position = state;
    }

    void set_safety_status(const safety_status_state &state)
    {
        g_safety_status = state;
    }

    void set_power_status(const power_status_state &state)
    {
        g_power_status = state;
    }

    local_position_state get_local_position()
    {
        return g_local_position;
    }

    safety_status_state get_safety_status()
    {
        return g_safety_status;
    }

    power_status_state get_power_status()
    {
        return g_power_status;
    }
}
