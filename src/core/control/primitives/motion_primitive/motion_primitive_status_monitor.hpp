#pragma once

#include <cstdint>

#include "../../../motion_mcu_communication/state/incoming/incoming_state.hpp"

namespace motion_primitive_status_monitor
{
    enum class expected_primitive_id : std::uint8_t
    {
        none = 0U,
        drive_forward = 2U,
        rotate_delta = 3U
    };

    struct snapshot
    {
        bool waiting = false;
        bool running = false;
        bool complete = false;
        bool success = false;
        bool failed = false;
        bool timed_out = false;
        expected_primitive_id expected_primitive = expected_primitive_id::none;
        std::uint32_t send_time_ms = 0U;
        std::uint32_t baseline_command_id = 0U;
        std::uint32_t tracked_command_id = 0U;
        motion_mcu_incoming_state::motion_primitive_status_state stm = {};
    };

    void init();
    void tick(std::uint32_t now_ms);

    void notify_drive_forward_sent(std::uint32_t now_ms);
    void notify_rotate_delta_sent(std::uint32_t now_ms);

    bool is_waiting();
    bool is_complete();
    bool was_successful();
    snapshot read_snapshot();
}
