#include "../incoming_request_handler_declarations.hpp"

#include "../../../middleware_parse_helpers.hpp"
#include "../../handler_helpers.hpp"
#include "../../../../../motion_mcu_communication/outgoing_payloads/service/obstacle_margin_control_payload.hpp"

namespace
{
    constexpr std::uint32_t timeout_us = 50000u;
}

namespace incoming_request_handlers
{
    bool handle_set_obstacle_margin_mm()
    {
        std::uint16_t margin_mm = 0U;
        const bool parsed_ok = middleware_parse_helpers::read_uint16_and_end(margin_mm, timeout_us);

        if (parsed_ok == false)
        {
            return false;
        }

        if (obstacle_margin_control_payload::send(margin_mm) == false)
        {
            return handler_helpers::write_response("rsp:fail(0)");
        }

        return handler_helpers::write_response("rsp:ok()");
    }
}
