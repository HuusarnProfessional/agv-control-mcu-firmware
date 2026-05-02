#include "../incoming_request_handler_declarations.hpp"

#include <cstdio>

#include "../../../middleware_parse_helpers.hpp"
#include "../../../../../motion_mcu_communication/motion_mcu_runtime.hpp"
#include "../../handler_helpers.hpp"

namespace
{
    constexpr std::uint32_t timeout_us = 50000u;
    constexpr std::int8_t valid_confidence = 100;
    constexpr std::int8_t invalid_confidence = 0;
    constexpr std::size_t response_capacity = 64u;
}

namespace incoming_request_handlers
{
    bool handle_get_position_local()
    {
        const bool parsed_ok = middleware_parse_helpers::read_end(timeout_us);

        if (parsed_ok == false)
        {
            return false;
        }

        const motion_mcu_runtime::local_position_state local_position = motion_mcu_runtime::get_local_position();
        const std::int8_t confidence = local_position.is_valid ? valid_confidence : invalid_confidence;
        char response[response_capacity] = {};
        const int formatted_length = std::snprintf(
            response,
            sizeof(response),
            "rsp:position_local(%d,%d,%d)",
            static_cast<int>(local_position.x_mm),
            static_cast<int>(local_position.y_mm),
            static_cast<int>(confidence));

        if ((formatted_length <= 0) || (static_cast<std::size_t>(formatted_length) >= sizeof(response)))
        {
            return false;
        }

        const bool response_written = handler_helpers::write_response_text(response);

        if (response_written == false)
        {
            return false;
        }

        return true;
    }
}
