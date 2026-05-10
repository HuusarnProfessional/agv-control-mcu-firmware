#include "motion_primitive_status_monitor.hpp"

namespace
{
    constexpr std::uint32_t start_timeout_ms = 1000U;
    constexpr std::uint32_t status_freshness_timeout_ms = 300U;
    constexpr std::uint8_t execution_state_complete = 3U;
    constexpr std::uint8_t error_code_none = 0U;

    motion_primitive_status_monitor::snapshot g_snapshot = {};

    void begin_wait(motion_primitive_status_monitor::expected_primitive_id expected_primitive, std::uint32_t now_ms)
    {
        g_snapshot = {};
        g_snapshot.waiting = true;
        g_snapshot.expected_primitive = expected_primitive;
        g_snapshot.send_time_ms = now_ms;
        g_snapshot.stm = motion_mcu_incoming_state::get_motion_primitive_status();

        if (g_snapshot.stm.valid == true)
        {
            g_snapshot.baseline_command_id = g_snapshot.stm.command_id;
        }
    }

    void finish_wait(bool success, bool timed_out)
    {
        g_snapshot.waiting = false;
        g_snapshot.running = false;
        g_snapshot.complete = true;
        g_snapshot.success = success;
        g_snapshot.failed = (success == false);
        g_snapshot.timed_out = timed_out;
    }
}

namespace motion_primitive_status_monitor
{
    void init()
    {
        g_snapshot = {};
    }

    void tick(std::uint32_t now_ms)
    {
        g_snapshot.stm = motion_mcu_incoming_state::get_motion_primitive_status();

        if ((g_snapshot.waiting == false) && (g_snapshot.running == false))
        {
            return;
        }

        if (g_snapshot.tracked_command_id == 0U)
        {
            const bool has_matching_start =
                (g_snapshot.stm.valid == true) &&
                (g_snapshot.stm.command_id != 0U) &&
                (g_snapshot.stm.command_id != g_snapshot.baseline_command_id) &&
                (g_snapshot.stm.active_primitive_id == static_cast<std::uint8_t>(g_snapshot.expected_primitive));

            if (has_matching_start == true)
            {
                g_snapshot.tracked_command_id = g_snapshot.stm.command_id;

                if (g_snapshot.stm.state == execution_state_complete)
                {
                    const bool success = g_snapshot.stm.failure_code == error_code_none;
                    finish_wait(success, false);
                    return;
                }

                g_snapshot.waiting = false;
                g_snapshot.running = true;
                return;
            }

            if ((now_ms - g_snapshot.send_time_ms) >= start_timeout_ms)
            {
                finish_wait(false, true);
            }

            return;
        }

        if ((g_snapshot.stm.valid == false) ||
            (g_snapshot.stm.command_id != g_snapshot.tracked_command_id))
        {
            if ((now_ms - g_snapshot.send_time_ms) >= start_timeout_ms)
            {
                finish_wait(false, true);
            }

            return;
        }

        if ((now_ms - g_snapshot.stm.received_time_ms) >= status_freshness_timeout_ms)
        {
            finish_wait(false, true);
            return;
        }

        if (g_snapshot.stm.state == execution_state_complete)
        {
            const bool success = g_snapshot.stm.failure_code == error_code_none;
            finish_wait(success, false);
            return;
        }

        g_snapshot.waiting = false;
        g_snapshot.running = true;
    }

    void notify_drive_forward_sent(std::uint32_t now_ms)
    {
        begin_wait(expected_primitive_id::drive_forward, now_ms);
    }

    void notify_rotate_delta_sent(std::uint32_t now_ms)
    {
        begin_wait(expected_primitive_id::rotate_delta, now_ms);
    }

    bool is_waiting()
    {
        return g_snapshot.waiting || g_snapshot.running;
    }

    bool is_complete()
    {
        return g_snapshot.complete;
    }

    bool was_successful()
    {
        return g_snapshot.complete && g_snapshot.success;
    }

    snapshot read_snapshot()
    {
        return g_snapshot;
    }
}
