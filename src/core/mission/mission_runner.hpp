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

    struct snapshot
    {
        bool is_running = false;
        bool start_pending = false;
        bool branch_changed = false;
        bool started_this_tick = false;
        std::uint16_t current_part = 0U;
        std::uint16_t part_count = 0U;
        std::uint8_t pending_branch_id = 0U;
        std::uint32_t pending_start_time_ms = 0U;
        std::uint32_t branch_changed_time_ms = 0U;
    };

    void init();

    void tick(std::uint32_t now_ms);

    bool started_this_tick();

    runner_status start_mission(const char *mission_id);

    runner_status abort_mission();

    bool is_running();

    snapshot read_snapshot();

    bool get_current_part(std::uint16_t &part_number_out);

    bool complete_current_part();
}
