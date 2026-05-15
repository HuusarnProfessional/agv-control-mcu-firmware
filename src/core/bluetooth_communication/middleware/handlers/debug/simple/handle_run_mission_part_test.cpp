#include "../debug_handler_declarations.hpp"

#include "../../../middleware_parse_helpers.hpp"
#include "../../../route/middleware_route_table.hpp"
#include "../../handler_helpers.hpp"
#include "../../../../../mission/mission_buffer.hpp"
#include "../../../../../mission/mission_runner.hpp"
#include "../../../../../mission/mission_transfer.hpp"
#include "debug_handler_helpers.hpp"

namespace
{
    constexpr const char *mission_id = "mission_part_test";
    constexpr std::uint16_t mission_part_count = 3U;
    constexpr std::uint32_t first_pause_duration_ms = 1000U;
    constexpr std::uint32_t second_pause_duration_ms = 1000U;
    constexpr std::uint32_t third_pause_duration_ms = 3000U;

    bool advance_internal_part_request()
    {
        if (mission_transfer::has_pending_request() == false)
        {
            return false;
        }

        char request_text[mission_transfer::max_request_text_length] = {};

        if (mission_transfer::pop_pending_request(request_text, sizeof(request_text)) == false)
        {
            return false;
        }

        mission_transfer::handle_response_ok();
        return true;
    }

    bool build_command_view(
        const char *command_name,
        const char *argument_stream,
        mission_buffer::mission_command_view &command_view_out)
    {
        if ((command_name == nullptr) || (argument_stream == nullptr))
        {
            return false;
        }

        const middleware_route_types::middleware_command_route *route =
            middleware_route_table::find_incoming_request_route(command_name);

        if (route == nullptr)
        {
            return false;
        }

        command_view_out = {};
        command_view_out.route = route;

        std::size_t index = 0U;

        while (argument_stream[index] != '\0')
        {
            if ((index + 1U) >= sizeof(command_view_out.argument_stream))
            {
                return false;
            }

            command_view_out.argument_stream[index] = argument_stream[index];
            ++index;
        }

        command_view_out.argument_stream[index] = '\0';
        return true;
    }

    bool append_pause_part(std::uint16_t part_number, std::uint32_t pause_duration_ms)
    {
        mission_buffer::mission_command_view start_command = {};
        mission_buffer::mission_command_view end_command = {};
        char pause_argument_stream[16] = {};

        if (build_command_view("dummy", ")", start_command) == false)
        {
            return false;
        }

        const int pause_argument_length = std::snprintf(
            pause_argument_stream,
            sizeof(pause_argument_stream),
            "%lu)",
            static_cast<unsigned long>(pause_duration_ms));

        if ((pause_argument_length <= 0) || (static_cast<std::size_t>(pause_argument_length) >= sizeof(pause_argument_stream)))
        {
            return false;
        }

        if (build_command_view("set_pause_ms", pause_argument_stream, end_command) == false)
        {
            return false;
        }

        if (advance_internal_part_request() == false)
        {
            return false;
        }

        const mission_transfer::transfer_status status =
            mission_transfer::set_part_info(
                mission_id,
                part_number,
                start_command,
                end_command,
                0U);

        return status == mission_transfer::transfer_status::ok;
    }
}

namespace debug_handlers
{
    bool handle_run_mission_part_test()
    {
        if (middleware_parse_helpers::read_end(debug_handler_helpers::timeout_us) == false)
        {
            return debug_handler_helpers::write_bad_format();
        }

        (void)mission_runner::abort_mission();
        mission_transfer::init();

        const mission_transfer::transfer_status begin_status =
            mission_transfer::begin_new_mission(mission_id, mission_part_count);

        if (begin_status != mission_transfer::transfer_status::ok)
        {
            return handler_helpers::write_response_text("err mission_part_test_begin");
        }

        if (append_pause_part(0U, first_pause_duration_ms) == false)
        {
            return handler_helpers::write_response_text("err mission_part_test_part0");
        }

        if (append_pause_part(1U, second_pause_duration_ms) == false)
        {
            return handler_helpers::write_response_text("err mission_part_test_part1");
        }

        if (append_pause_part(2U, third_pause_duration_ms) == false)
        {
            return handler_helpers::write_response_text("err mission_part_test_part2");
        }

        if (mission_transfer::is_transfer_complete() == false)
        {
            return handler_helpers::write_response_text("err mission_part_test_incomplete");
        }

        const mission_runner::runner_status start_status = mission_runner::start_mission(mission_id);

        if (start_status != mission_runner::runner_status::ok)
        {
            return handler_helpers::write_response_text("err mission_part_test_start");
        }

        char response[96] = {};
        const int response_length = std::snprintf(
            response,
            sizeof(response),
            "ok mission_part_test_started parts %u pauses %lu,%lu,%lu",
            static_cast<unsigned>(mission_part_count),
            static_cast<unsigned long>(first_pause_duration_ms),
            static_cast<unsigned long>(second_pause_duration_ms),
            static_cast<unsigned long>(third_pause_duration_ms));

        if ((response_length <= 0) || (static_cast<std::size_t>(response_length) >= sizeof(response)))
        {
            return false;
        }

        return handler_helpers::write_response_text(response);
    }
}
