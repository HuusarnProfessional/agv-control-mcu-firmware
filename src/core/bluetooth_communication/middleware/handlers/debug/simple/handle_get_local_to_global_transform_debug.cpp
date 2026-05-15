#include "../debug_handler_declarations.hpp"

#include <Arduino.h>

#include <cstddef>
#include <cstdio>

#include "../../../middleware_parse_helpers.hpp"
#include "../../handler_helpers.hpp"
#include "../../../../../motion_mcu_communication/state/incoming/incoming_state.hpp"
#include "../../../../../position_sensorfusion/local_to_global_transform/local_to_global_transform.hpp"
#include "debug_handler_helpers.hpp"

namespace
{
    std::uint32_t bool_to_u32(bool value)
    {
        if (value == true)
        {
            return 1U;
        }

        return 0U;
    }
}

namespace debug_handlers
{
    bool handle_get_local_to_global_transform_debug()
    {
        if (middleware_parse_helpers::read_end(debug_handler_helpers::timeout_us) == false)
        {
            return debug_handler_helpers::write_bad_format();
        }

        const motion_mcu_incoming_state::local_position_state local_position = motion_mcu_incoming_state::get_local_position();
        const std::uint32_t now_ms = static_cast<std::uint32_t>(millis());
        const local_to_global_transform::output_snapshot state = local_to_global_transform::read_output(local_position, now_ms);

        char response[512] = {};
        const int length = std::snprintf(response, sizeof(response), "local_to_global_transform_debug has_pose %lu has_transform %lu branch_matches %lu transform_activated %lu x_um %lld y_um %lld heading_urad %ld confidence_position %u confidence_heading %u pose_id %u branch_id %u reference_sample_id %lu transform_confidence_position %u transform_confidence_heading %u is_mission_seed %lu activation_time_ms %lu", static_cast<unsigned long>(bool_to_u32(state.has_pose)), static_cast<unsigned long>(bool_to_u32(state.has_transform)), static_cast<unsigned long>(bool_to_u32(state.branch_matches)), static_cast<unsigned long>(bool_to_u32(state.transform_activated)), static_cast<long long>(state.x_um), static_cast<long long>(state.y_um), static_cast<long>(state.heading_urad), static_cast<unsigned>(state.confidence_position), static_cast<unsigned>(state.confidence_heading), static_cast<unsigned>(state.pose_id), static_cast<unsigned>(state.branch_id), static_cast<unsigned long>(state.reference_sample_id), static_cast<unsigned>(state.transform_confidence_position), static_cast<unsigned>(state.transform_confidence_heading), static_cast<unsigned long>(bool_to_u32(state.is_mission_seed)), static_cast<unsigned long>(state.activation_time_ms));

        if ((length <= 0) || (static_cast<std::size_t>(length) >= sizeof(response)))
        {
            return false;
        }

        return handler_helpers::write_response_text(response);
    }
}
