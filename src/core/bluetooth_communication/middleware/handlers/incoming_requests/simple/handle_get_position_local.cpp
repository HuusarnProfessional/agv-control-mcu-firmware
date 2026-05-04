#include "../incoming_request_handler_declarations.hpp"

#include <cstdio>

#include "../../../middleware_parse_helpers.hpp"
#include "../../../../../position_sensorfusion/position_sensorfusion.hpp"
#include "../../handler_helpers.hpp"

namespace
{
    constexpr std::uint32_t timeout_us = 50000u;
    constexpr std::uint16_t stm_confidence_max = 1000U;
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

        const position_sensorfusion::output_snapshot local_position = position_sensorfusion::read_output();
        const std::int32_t x_mm = static_cast<std::int32_t>(local_position.x_um / 1000LL);
        const std::int32_t y_mm = static_cast<std::int32_t>(local_position.y_um / 1000LL);
        std::int32_t confidence = 0;

        if (local_position.has_pose == true)
        {
            std::uint16_t confidence_clamped = local_position.confidence_position;

            if (confidence_clamped > stm_confidence_max)
            {
                confidence_clamped = stm_confidence_max;
            }

            confidence = static_cast<std::int32_t>((static_cast<std::uint32_t>(confidence_clamped) * 100U) / stm_confidence_max);
        }

        char response[response_capacity] = {};
        const int formatted_length = std::snprintf(
            response,
            sizeof(response),
            "rsp:position_local(%d,%d,%d)",
            static_cast<int>(x_mm),
            static_cast<int>(y_mm),
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
