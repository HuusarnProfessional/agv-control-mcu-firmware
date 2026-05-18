#include "mission_pipeline.hpp"

#include <Arduino.h>

#include "mission_buffer.hpp"
#include "mission_runner.hpp"
#include "mission_transfer.hpp"
#include "../control/primitives/motion_primitive/motion_primitive_status_monitor.hpp"
#include "../control/primitives/pause/pause_pipeline.hpp"
#include "../bluetooth_communication/middleware/middleware_handler_input_bridge.hpp"
#include "../pure_pursuit/pure_pursuit.hpp"

namespace
{
    enum class part_stage : std::uint8_t
    {
        idle = 0u,
        waiting_before_path_start,
        path_running,
        waiting_before_part_complete
    };

    part_stage g_part_stage = part_stage::idle;
    std::uint16_t g_stage_part_number = 0U;
    mission_buffer::mission_part_view g_part_view = {};
    bool g_waiting_for_motion_primitive = false;

    void log_pass(const char *step_name)
    {
        Serial.print("mission ");
        Serial.print(step_name);
        Serial.println(" pass");
    }

    void log_fail(const char *step_name)
    {
        Serial.print("mission ");
        Serial.print(step_name);
        Serial.println(" fail");
    }

    bool run_command(const mission_buffer::mission_command_view &command_view)
    {
        if (command_view.route == nullptr)
        {
            return false;
        }

        middleware_handler_input_bridge::set_memory_input(command_view.argument_stream);
        const bool command_ok = command_view.route->handler();
        middleware_handler_input_bridge::clear();

        return command_ok;
    }

    void reset_part_execution_state()
    {
        g_part_stage = part_stage::idle;
        g_stage_part_number = 0U;
        g_part_view = {};
        g_waiting_for_motion_primitive = false;
        pure_pursuit::stop();
    }

    void abort_mission_and_reset(mission_runner::abort_reason reason)
    {
        mission_runner::abort_mission(reason);
        reset_part_execution_state();
        log_fail("abort_mission_and_reset");
    }

    bool complete_part_or_wait_for_pause()
    {
        if (pause_pipeline::is_active() == true)
        {
            log_pass("complete_part_or_wait_for_pause pause");
            g_waiting_for_motion_primitive = false;
            g_part_stage = part_stage::waiting_before_part_complete;
            return true;
        }

        if (motion_primitive_status_monitor::is_waiting() == true)
        {
            log_pass("complete_part_or_wait_for_pause primitive");
            g_waiting_for_motion_primitive = true;
            g_part_stage = part_stage::waiting_before_part_complete;
            return true;
        }

        const bool complete_ok = mission_runner::complete_current_part();

        if (complete_ok == true)
        {
            log_pass("complete_current_part");
        }
        else
        {
            log_fail("complete_current_part");
        }

        return complete_ok;
    }

    bool start_path_or_run_end_command(std::uint16_t current_part)
    {
        if (g_part_view.path_chunk_count > 0u)
        {
            const bool path_started = pure_pursuit::start_part(current_part);

            if (path_started == false)
            {
                log_fail("start_path");
                return false;
            }

            g_part_stage = part_stage::path_running;
            log_pass("start_path");
            return true;
        }

        const bool end_command_ok = run_command(g_part_view.end_command);

        if (end_command_ok == false)
        {
            log_fail("end_command");
            return false;
        }

        log_pass("end_command");
        return complete_part_or_wait_for_pause();
    }

    void tick_waiting_before_path_start(std::uint16_t current_part)
    {
        if (pause_pipeline::is_active() == true)
        {
            log_pass("tick_waiting_before_path_start pause");
            return;
        }

        if (g_waiting_for_motion_primitive == true)
        {
            const motion_primitive_status_monitor::snapshot primitive_snapshot = motion_primitive_status_monitor::read_snapshot();

            if (motion_primitive_status_monitor::is_waiting() == true)
            {
                log_pass("tick_waiting_before_path_start primitive");
                return;
            }

            if ((motion_primitive_status_monitor::is_complete() == true) &&
                (motion_primitive_status_monitor::was_successful() == false))
            {
                log_fail("tick_waiting_before_path_start primitive");
                abort_mission_and_reset(mission_runner::abort_reason::motion_primitive_failed);
                return;
            }

            (void)primitive_snapshot;
            g_waiting_for_motion_primitive = false;
        }

        const bool continued_ok = start_path_or_run_end_command(current_part);

        if (continued_ok == false)
        {
            abort_mission_and_reset(mission_runner::abort_reason::end_command_failed);
        }
    }

    void tick_path_running()
    {
        const pure_pursuit::snapshot path_snapshot = pure_pursuit::read_snapshot();

        if (path_snapshot.active == true)
        {
            return;
        }

        if (path_snapshot.complete == false)
        {
            return;
        }

        if (path_snapshot.success == false)
        {
            log_fail("path_running");
            abort_mission_and_reset(mission_runner::abort_reason::path_failed);
            return;
        }

        log_pass("path_running");

        const bool end_command_ok = run_command(g_part_view.end_command);

        if (end_command_ok == false)
        {
            log_fail("end_command");
            abort_mission_and_reset(mission_runner::abort_reason::end_command_failed);
            return;
        }

        log_pass("end_command");

        const bool complete_ok = complete_part_or_wait_for_pause();

        if (complete_ok == false)
        {
            abort_mission_and_reset(mission_runner::abort_reason::part_complete_failed);
            return;
        }

        if (g_part_stage == part_stage::waiting_before_part_complete)
        {
            return;
        }

        reset_part_execution_state();
    }

    void tick_waiting_before_part_complete()
    {
        if (pause_pipeline::is_active() == true)
        {
            log_pass("tick_waiting_before_part_complete pause");
            return;
        }

        if (g_waiting_for_motion_primitive == true)
        {
            if (motion_primitive_status_monitor::is_waiting() == true)
            {
                log_pass("tick_waiting_before_part_complete primitive");
                return;
            }

            if ((motion_primitive_status_monitor::is_complete() == true) &&
                (motion_primitive_status_monitor::was_successful() == false))
            {
                log_fail("tick_waiting_before_part_complete primitive");
                abort_mission_and_reset(mission_runner::abort_reason::motion_primitive_failed);
                return;
            }

            g_waiting_for_motion_primitive = false;
        }

        const bool complete_ok = mission_runner::complete_current_part();

        if (complete_ok == false)
        {
            abort_mission_and_reset(mission_runner::abort_reason::part_complete_failed);
            return;
        }

        reset_part_execution_state();
    }

    void begin_current_part(std::uint16_t current_part)
    {
        const bool has_part_info = mission_buffer::get_part_info(current_part, g_part_view);

        if (has_part_info == false)
        {
            log_fail("get_part_info");
            abort_mission_and_reset(mission_runner::abort_reason::part_info_missing);
            return;
        }

        g_stage_part_number = current_part;
        const bool start_command_ok = run_command(g_part_view.start_command);

        if (start_command_ok == false)
        {
            log_fail("start_command");
            abort_mission_and_reset(mission_runner::abort_reason::start_command_failed);
            return;
        }

        log_pass("start_command");

        if (pause_pipeline::is_active() == true)
        {
            log_pass("begin_current_part pause");
            g_waiting_for_motion_primitive = false;
            g_part_stage = part_stage::waiting_before_path_start;
            return;
        }

        if (motion_primitive_status_monitor::is_waiting() == true)
        {
            log_pass("begin_current_part primitive");
            g_waiting_for_motion_primitive = true;
            g_part_stage = part_stage::waiting_before_path_start;
            return;
        }

        const bool continued_ok = start_path_or_run_end_command(current_part);

        if (continued_ok == false)
        {
            abort_mission_and_reset(mission_runner::abort_reason::end_command_failed);
            return;
        }

        if (g_part_stage == part_stage::path_running)
        {
            return;
        }

        if (g_part_stage == part_stage::waiting_before_part_complete)
        {
            return;
        }

        reset_part_execution_state();
    }
}

namespace mission_pipeline
{
    void init()
    {
        mission_transfer::init();
        mission_runner::init();
        reset_part_execution_state();
    }

    void tick(std::uint32_t now_ms)
    {
        mission_runner::tick(now_ms);

        if (mission_runner::started_this_tick() == true)
        {
            return;
        }

        std::uint16_t current_part = 0u;
        const bool has_current_part = mission_runner::get_current_part(current_part);

        if (has_current_part == false)
        {
            reset_part_execution_state();
            return;
        }

        if ((g_part_stage != part_stage::idle) && (g_stage_part_number != current_part))
        {
            reset_part_execution_state();
        }

        if (g_part_stage == part_stage::waiting_before_path_start)
        {
            tick_waiting_before_path_start(current_part);
            return;
        }

        if (g_part_stage == part_stage::path_running)
        {
            tick_path_running();
            return;
        }

        if (g_part_stage == part_stage::waiting_before_part_complete)
        {
            tick_waiting_before_part_complete();
            return;
        }

        begin_current_part(current_part);
    }
}
