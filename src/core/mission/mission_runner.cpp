#include "mission_runner.hpp"

#include <Arduino.h>

#include <cstddef>
#include <cstdio>

#include "mission_buffer.hpp"
#include "mission_transfer.hpp"
#include "../bluetooth_communication/middleware/handlers/handler_helpers.hpp"
#include "../motion_mcu_communication/outgoing_payloads/service/position_correction_payload.hpp"
#include "../motion_mcu_communication/state/incoming/incoming_state.hpp"
#include "../position_sensorfusion/position_sensorfusion_pipeline.hpp"

namespace
{
    constexpr std::uint32_t branch_settle_time_ms = 200U;
    constexpr std::uint32_t branch_start_timeout_ms = 1500U;

    bool g_is_running = false;
    bool g_start_pending = false;
    bool g_branch_changed = false;
    bool g_started_this_tick = false;
    std::uint16_t g_current_part = 0u;
    std::uint8_t g_pending_branch_id = 0U;
    std::uint32_t g_pending_start_time_ms = 0U;
    std::uint32_t g_branch_changed_time_ms = 0U;

    void clear_pending_start()
    {
        g_start_pending = false;
        g_branch_changed = false;
        g_pending_branch_id = 0U;
        g_pending_start_time_ms = 0U;
        g_branch_changed_time_ms = 0U;
    }

    void begin_running()
    {
        g_is_running = true;
        g_started_this_tick = true;
        g_current_part = 0u;
        clear_pending_start();
    }

    void send_mission_complete_event(std::uint16_t completed_part, std::uint16_t part_count)
    {
        char response[96] = {};
        const int formatted_length = std::snprintf(response, sizeof(response), "rsp:mission_complete(%u,%u)", static_cast<unsigned int>(completed_part), static_cast<unsigned int>(part_count));

        if ((formatted_length <= 0) || (static_cast<std::size_t>(formatted_length) >= sizeof(response)))
        {
            return;
        }

        (void)handler_helpers::write_response_text(response);
    }

    void send_mission_aborted_event(std::uint16_t current_part, std::uint16_t part_count, mission_runner::abort_reason reason)
    {
        char response[96] = {};
        const int formatted_length = std::snprintf(response, sizeof(response), "rsp:mission_aborted(%u,%u,%u)", static_cast<unsigned int>(current_part), static_cast<unsigned int>(part_count), static_cast<unsigned int>(reason));

        if ((formatted_length <= 0) || (static_cast<std::size_t>(formatted_length) >= sizeof(response)))
        {
            return;
        }

        (void)handler_helpers::write_response_text(response);
    }
}

namespace mission_runner
{
    void init()
    {
        g_is_running = false;
        g_current_part = 0u;
        clear_pending_start();
    }

    void tick(std::uint32_t now_ms)
    {
        g_started_this_tick = false;

        if (g_start_pending == false)
        {
            return;
        }

        const motion_mcu_incoming_state::local_position_state local_position = motion_mcu_incoming_state::get_local_position();

        if (local_position.has_pose == false)
        {
            if ((now_ms - g_pending_start_time_ms) >= branch_start_timeout_ms)
            {
                clear_pending_start();
            }

            return;
        }

        if (g_branch_changed == false)
        {
            if (local_position.branch_id != g_pending_branch_id)
            {
                g_branch_changed = true;
                g_branch_changed_time_ms = now_ms;
                return;
            }

            if ((now_ms - g_pending_start_time_ms) >= branch_start_timeout_ms)
            {
                begin_running();
            }

            return;
        }

        if ((now_ms - g_branch_changed_time_ms) < branch_settle_time_ms)
        {
            return;
        }

        begin_running();
    }

    bool started_this_tick()
    {
        return g_started_this_tick;
    }

    runner_status start_mission(const char *mission_id)
    {
        if (mission_id == nullptr)
        {
            return runner_status::invalid_arg;
        }

        if (mission_transfer::has_active_mission() == false)
        {
            return runner_status::no_mission;
        }

        if (mission_transfer::mission_matches(mission_id) == false)
        {
            return runner_status::mission_mismatch;
        }

        if (mission_transfer::is_transfer_complete() == false)
        {
            return runner_status::mission_not_ready;
        }

        const motion_mcu_incoming_state::local_position_state local_position = motion_mcu_incoming_state::get_local_position();

        if (local_position.has_pose == false)
        {
            return runner_status::mission_not_ready;
        }

        if ((position_sensorfusion_pipeline::is_local_only_mode_enabled() == true) || (position_sensorfusion_pipeline::is_global_anchor_test_mode_enabled() == true))
        {
            begin_running();
            return runner_status::ok;
        }

        if (position_correction_payload::send(local_position.pose_id, local_position.branch_id) == false)
        {
            return runner_status::mission_not_ready;
        }

        g_is_running = false;
        g_current_part = 0u;
        g_start_pending = true;
        g_branch_changed = false;
        g_pending_branch_id = local_position.branch_id;
        g_pending_start_time_ms = static_cast<std::uint32_t>(millis());
        g_branch_changed_time_ms = 0U;

        return runner_status::ok;
    }

    runner_status abort_mission()
    {
        return abort_mission(abort_reason::manual);
    }

    runner_status abort_mission(abort_reason reason)
    {
        if (mission_transfer::has_active_mission() == false)
        {
            return runner_status::no_mission;
        }

        const std::uint16_t current_part = g_current_part;
        const std::uint16_t part_count = mission_buffer::part_count();

        g_is_running = false;
        g_current_part = 0u;
        clear_pending_start();
        send_mission_aborted_event(current_part, part_count, reason);

        return runner_status::ok;
    }

    bool is_running()
    {
        return g_is_running;
    }

    snapshot read_snapshot()
    {
        snapshot state = {};

        state.is_running = g_is_running;
        state.start_pending = g_start_pending;
        state.branch_changed = g_branch_changed;
        state.started_this_tick = g_started_this_tick;
        state.current_part = g_current_part;
        state.part_count = mission_buffer::part_count();
        state.pending_branch_id = g_pending_branch_id;
        state.pending_start_time_ms = g_pending_start_time_ms;
        state.branch_changed_time_ms = g_branch_changed_time_ms;

        return state;
    }

    bool get_current_part(std::uint16_t &part_number_out)
    {
        if (g_is_running == false)
        {
            return false;
        }

        part_number_out = g_current_part;
        return true;
    }

    bool complete_current_part()
    {
        if (g_is_running == false)
        {
            return false;
        }

        const std::uint16_t part_count = mission_buffer::part_count();

        if (part_count == 0u)
        {
            g_is_running = false;
            g_current_part = 0u;
            Serial.println("mission completed");
            send_mission_complete_event(0U, part_count);
           
            return true;
        }

        if ((g_current_part + 1u) < part_count)
        {
            ++g_current_part;
            return true;
        }

        g_is_running = false;
        Serial.println("mission completed");
        send_mission_complete_event(g_current_part, part_count);
        
        return true;
    }
}
