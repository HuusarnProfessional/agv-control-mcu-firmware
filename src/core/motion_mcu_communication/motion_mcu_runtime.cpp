#include "motion_mcu_runtime.hpp"

namespace
{
    motion_mcu_runtime::local_position_state g_local_position = {};
}

namespace motion_mcu_runtime
{
    void init()
    {
        g_local_position = {};
    }

    void set_local_position(const local_position_state &state)
    {
        g_local_position = state;
    }

    local_position_state get_local_position()
    {
        return g_local_position;
    }
}
