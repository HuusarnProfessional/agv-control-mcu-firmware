#include "../incoming_request_handler_declarations.hpp"

#include <cstdio>

#include "../../../middleware_parse_helpers.hpp"
#include "../../../../../motion_mcu_communication/state/incoming/incoming_state.hpp"
#include "../../handler_helpers.hpp"

namespace
{
    constexpr std::uint32_t timeout_us = 50000u;
    constexpr std::uint16_t stm_confidence_max = 1000U;
    constexpr std::size_t response_capacity = 96u;
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

        const motion_mcu_incoming_state::local_position_state local_position = motion_mcu_incoming_state::get_local_position();
        const std::int32_t x_mm = static_cast<std::int32_t>(local_position.x_um / 1000LL);
        const std::int32_t y_mm = static_cast<std::int32_t>(local_position.y_um / 1000LL);
        const double heading_deg = static_cast<double>(local_position.heading_urad) * 180.0 / 3141592.653589793;
        std::int32_t confidence_position = 0;
        std::int32_t confidence_heading = 0;

        if (local_position.has_pose == true)
        {
            std::uint16_t confidence_position_clamped = local_position.confidence_position;
            std::uint16_t confidence_heading_clamped = local_position.confidence_heading;

            if (confidence_position_clamped > stm_confidence_max)
            {
                confidence_position_clamped = stm_confidence_max;
            }

            if (confidence_heading_clamped > stm_confidence_max)
            {
                confidence_heading_clamped = stm_confidence_max;
            }

            confidence_position =
                static_cast<std::int32_t>((static_cast<std::uint32_t>(confidence_position_clamped) * 100U) / stm_confidence_max);

            confidence_heading =
                static_cast<std::int32_t>((static_cast<std::uint32_t>(confidence_heading_clamped) * 100U) / stm_confidence_max);
        }

        char response[response_capacity] = {};
        const int formatted_length = std::snprintf(
            response,
            sizeof(response),
            "rsp:position_local(%d,%d,%.2f,%d,%d,%u,%u)",
            static_cast<int>(x_mm),
            static_cast<int>(y_mm),
            heading_deg,
            static_cast<int>(confidence_position),
            static_cast<int>(confidence_heading),
            static_cast<unsigned int>(local_position.pose_id),
            static_cast<unsigned int>(local_position.branch_id));

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
