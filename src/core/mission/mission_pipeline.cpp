#include "mission_pipeline.hpp"

#include "mission_buffer.hpp"
#include "mission_runner.hpp"
#include "mission_transfer.hpp"
#include "../bluetooth_communication/middleware/middleware_handler_input_bridge.hpp"

namespace
{
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
}

namespace mission_pipeline
{
    void init()
    {
        mission_transfer::init();
        mission_runner::init();
    }

    void tick(std::uint32_t now_ms)
    {
        (void)now_ms;

        std::uint16_t current_part = 0u;
        const bool has_current_part = mission_runner::get_current_part(current_part);

        if (has_current_part == false)
        {
            return;
        }

        mission_buffer::mission_part_view part_view = {};
        const bool has_part_info = mission_buffer::get_part_info(current_part, part_view);

        if (has_part_info == false)
        {
            mission_runner::abort_mission();
            return;
        }

        run_command(part_view.start_command);
        run_command(part_view.end_command);

        mission_runner::complete_current_part();
    }
}
