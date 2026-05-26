#include "../debug_handler_declarations.hpp"

#include <cstddef>
#include <cstdio>

#include "../../../middleware_parse_helpers.hpp"
#include "../../handler_helpers.hpp"
#include "../../../../../position_trace/position_trace_logger.hpp"
#include "debug_handler_helpers.hpp"

namespace
{
    constexpr std::size_t response_capacity = 1024U;
}

namespace debug_handlers
{
    bool handle_clear_position_trace()
    {
        if (middleware_parse_helpers::read_end(debug_handler_helpers::timeout_us) == false)
        {
            return debug_handler_helpers::write_bad_format();
        }

        position_trace_logger::clear();
        return handler_helpers::write_response_text("position_trace_cleared");
    }

    bool handle_set_position_trace_enabled()
    {
        bool enabled = false;

        if (middleware_parse_helpers::read_bool_and_end(enabled, debug_handler_helpers::timeout_us) == false)
        {
            return debug_handler_helpers::write_bad_format();
        }

        position_trace_logger::set_enabled(enabled);

        if (enabled == true)
        {
            return handler_helpers::write_response_text("position_trace_enabled 1");
        }

        return handler_helpers::write_response_text("position_trace_enabled 0");
    }

    bool handle_set_position_trace_period_ms()
    {
        std::uint16_t period_ms = 0U;

        if (middleware_parse_helpers::read_uint16_and_end(period_ms, debug_handler_helpers::timeout_us) == false)
        {
            return debug_handler_helpers::write_bad_format();
        }

        position_trace_logger::set_period_ms(period_ms);

        char response[64] = {};
        const int length = std::snprintf(
            response,
            sizeof(response),
            "position_trace_period_ms %u",
            static_cast<unsigned>(position_trace_logger::get_period_ms()));

        if ((length <= 0) || (static_cast<std::size_t>(length) >= sizeof(response)))
        {
            return false;
        }

        return handler_helpers::write_response_text(response);
    }

    bool handle_get_position_trace_status()
    {
        if (middleware_parse_helpers::read_end(debug_handler_helpers::timeout_us) == false)
        {
            return debug_handler_helpers::write_bad_format();
        }

        char response[128] = {};

        if (position_trace_logger::format_status(response, sizeof(response)) == false)
        {
            return false;
        }

        return handler_helpers::write_response_text(response);
    }

    bool handle_get_position_trace_packet()
    {
        if (middleware_parse_helpers::read_end(debug_handler_helpers::timeout_us) == false)
        {
            return debug_handler_helpers::write_bad_format();
        }

        char response[response_capacity] = {};

        if (position_trace_logger::format_packet(response, sizeof(response)) == false)
        {
            return false;
        }

        return handler_helpers::write_response_text(response);
    }
}
