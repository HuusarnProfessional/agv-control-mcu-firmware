#pragma once

#include <cstdint>

namespace motion_mcu_runtime
{
    struct local_position_state
    {
        bool has_pose = false;
        std::int64_t x_um = 0;
        std::int64_t y_um = 0;
        std::int32_t heading_urad = 0;
        std::uint16_t confidence_position = 0U;
        std::uint16_t confidence_heading = 0U;
        std::uint8_t pose_id = 0U;
        std::uint8_t branch_id = 0U;
    };

    struct safety_status_state
    {
        bool fault_latched = false;
        std::uint32_t controller_time_ms = 0U;
    };

    struct power_status_state
    {
        std::uint32_t voltage_mv = 0U;
        std::uint32_t sample_time_ms = 0U;
        std::uint8_t status = 0U;
    };

    void init();

    void set_local_position(const local_position_state &state);
    void set_safety_status(const safety_status_state &state);
    void set_power_status(const power_status_state &state);

    local_position_state get_local_position();
    safety_status_state get_safety_status();
    power_status_state get_power_status();
}
