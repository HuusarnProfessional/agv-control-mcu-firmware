#include "../incoming_request_handler_declarations.hpp"

#include <Arduino.h>

#include <cstddef>
#include <cstdint>
#include <cstdio>

#include "../../../middleware_parse_helpers.hpp"
#include "../../../../../position_sensorfusion/filtered_global_position/filtered_global_position.hpp"
#include "../../handler_helpers.hpp"

namespace
{
    constexpr std::uint32_t timeout_us = 50000u;
    constexpr std::uint16_t confidence_max = 1000U;
    constexpr std::size_t response_capacity = 128u;

    std::int32_t confidence_to_percent(std::uint16_t confidence, bool valid)
    {
        if (valid == false)
        {
            return 0;
        }

        if (confidence > confidence_max)
        {
            confidence = confidence_max;
        }

        return static_cast<std::int32_t>((static_cast<std::uint32_t>(confidence) * 100U) / confidence_max);
    }
}

namespace incoming_request_handlers
{
    bool handle_get_position_filtered_global()
    {
        const bool parsed_ok = middleware_parse_helpers::read_end(timeout_us);

        if (parsed_ok == false)
        {
            return false;
        }

        const std::uint32_t now_ms = static_cast<std::uint32_t>(millis());
        const filtered_global_position::output_snapshot filtered_position = filtered_global_position::read_output(now_ms);
        const std::int32_t x_mm = static_cast<std::int32_t>(filtered_position.x_um / 1000LL);
        const std::int32_t y_mm = static_cast<std::int32_t>(filtered_position.y_um / 1000LL);
        const double heading_deg = static_cast<double>(filtered_position.heading_urad) * 180.0 / 3141592.653589793;
        const std::int32_t confidence_position = confidence_to_percent(filtered_position.confidence_position, filtered_position.has_position);
        const std::int32_t confidence_heading = confidence_to_percent(filtered_position.confidence_heading, filtered_position.has_heading);

        char response[response_capacity] = {};
        const int formatted_length = std::snprintf(
            response,
            sizeof(response),
            "rsp:position_filtered_global(%d,%d,%.2f,%d,%d,%lu,%lu)",
            static_cast<int>(x_mm),
            static_cast<int>(y_mm),
            heading_deg,
            static_cast<int>(confidence_position),
            static_cast<int>(confidence_heading),
            static_cast<unsigned long>(filtered_position.sample_id),
            static_cast<unsigned long>(filtered_position.received_time_ms));

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
