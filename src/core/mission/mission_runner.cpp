#include "mission_runner.hpp"

#include <Arduino.h>

#include "mission_buffer.hpp"
#include "mission_transfer.hpp"

namespace
{
    bool g_is_running = false;
    std::uint16_t g_current_part = 0u;
}

namespace mission_runner
{
    void init()
    {
        g_is_running = false;
        g_current_part = 0u;
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

        g_is_running = true;
        g_current_part = 0u;

        return runner_status::ok;
    }

    runner_status abort_mission()
    {
        if (mission_transfer::has_active_mission() == false)
        {
            return runner_status::no_mission;
        }

        g_is_running = false;
        g_current_part = 0u;

        return runner_status::ok;
    }

    bool is_running()
    {
        return g_is_running;
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
           
            return true;
        }

        if ((g_current_part + 1u) < part_count)
        {
            ++g_current_part;
            return true;
        }

        g_is_running = false;
        Serial.println("mission completed");
        
        return true;
    }
}
