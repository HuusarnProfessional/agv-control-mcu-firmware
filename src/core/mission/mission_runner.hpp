#pragma once

#include <cstdint>

namespace mission_runner
{
    enum class runner_status : std::uint8_t
    {
        ok = 0,
        invalid_arg,
        no_mission,
        mission_mismatch,
        mission_not_ready,
        not_running
    };

    void init();

    runner_status start_mission(const char *mission_id);

    runner_status abort_mission();

    bool is_running();

    bool get_current_part(std::uint16_t &part_number_out);
}
