#include "../incoming_request_handler_declarations.hpp"

#include <cstdio>

#include "../../../middleware_parse_helpers.hpp"
#include "../../../../../global_positioning/global_position_api.hpp"
#include "../../handler_helpers.hpp"

namespace
{
    constexpr std::uint32_t timeout_us = 50000u;
    constexpr std::size_t response_capacity = 128u;
}

namespace incoming_request_handlers
{
    bool handle_get_position_global()
    {
        const bool parsed_ok = middleware_parse_helpers::read_end(timeout_us);

        if (parsed_ok == false)
        {
            return false;
        }

        global_position_api::global_position_sample sample = {};
        const bool has_sample = global_position_api::read_sample(sample);

        if (has_sample == false)
        {
            return handler_helpers::write_response("rsp:fail(1)");
        }

        char response[response_capacity] = {};
        const int formatted_length = std::snprintf(
            response,
            sizeof(response),
            "rsp:position_global(%ld,%ld,%ld,%u,%lu,%lu,%lu)",
            static_cast<long>(sample.x_mm),
            static_cast<long>(sample.y_mm),
            static_cast<long>(sample.z_mm),
            static_cast<unsigned int>(sample.quality_factor),
            static_cast<unsigned long>(sample.sample_id),
            static_cast<unsigned long>(sample.request_id),
            static_cast<unsigned long>(sample.received_time_ms));

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
