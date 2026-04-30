#pragma once

#include <cstdint>

namespace motion_mcu_runtime
{
    struct local_position_state
    {
        std::int16_t x_mm;
        std::int16_t y_mm;
        std::int16_t heading_mrad;
        bool is_valid;
    };

    void init();

    void set_local_position(const local_position_state &state);

    local_position_state get_local_position();
}
