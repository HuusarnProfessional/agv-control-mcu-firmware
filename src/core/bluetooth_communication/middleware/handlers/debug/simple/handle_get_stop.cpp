#include "../debug_handler_declarations.hpp"

#include <cstdint>

#include "../../../middleware_parse_helpers.hpp"
#include "../../handler_helpers.hpp"
#include "../../../../../motion_mcu_communication/state/incoming/incoming_state.hpp"

namespace
{
    constexpr std::uint32_t timeout_us = 50000U;
}

namespace debug_handlers
{
    bool handle_get_stop()
    {
        if (middleware_parse_helpers::read_end(timeout_us) == false)
        {
            return false;
        }

        const motion_mcu_incoming_state::safety_status_state safety_status = motion_mcu_incoming_state::get_safety_status();
        return safety_status.fault_latched
                   ? handler_helpers::write_response_text("stop 1")
                   : handler_helpers::write_response_text("stop 0");
    }
}
