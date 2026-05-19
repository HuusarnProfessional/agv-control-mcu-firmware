#include "../debug_handler_declarations.hpp"

#include <cstddef>
#include <cstdio>

#include "../../../middleware_parse_helpers.hpp"
#include "../../handler_helpers.hpp"
#include "../../../../../position_sensorfusion/local_to_global_transform/local_to_global_transform.hpp"
#include "debug_handler_helpers.hpp"

namespace
{
    std::uint32_t last_sent_anchor_event_id = 0U;
    std::uint32_t last_sent_anchor_change_id = 0U;

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
    bool handle_get_anchor_event_debug()
    {
        if (middleware_parse_helpers::read_end(debug_handler_helpers::timeout_us) == false)
        {
            return debug_handler_helpers::write_bad_format();
        }

        const local_to_global_transform::anchor_event_snapshot event = local_to_global_transform::read_anchor_event();

        if (event.valid == false)
        {
            return true;
        }

        if (event.event_id == last_sent_anchor_event_id)
        {
            return true;
        }

        last_sent_anchor_event_id = event.event_id;

        char response[900] = {};
        const int length = std::snprintf(response, sizeof(response), "anchor_event_debug event_id %lu type %u initial %lu mission_seed %lu source_pose_id %u source_branch_id %u activation_time_ms %lu reference_confidence %u reference_sample_id %lu local_x_um %lld local_y_um %lld local_heading_urad %ld global_x_um %lld global_y_um %lld global_heading_urad %ld rotation_urad %ld matrix_cos_ppm %ld matrix_sin_ppm %ld matrix_m00_ppm %ld matrix_m01_ppm %ld matrix_m10_ppm %ld matrix_m11_ppm %ld", static_cast<unsigned long>(event.event_id), static_cast<unsigned>(event.type), static_cast<unsigned long>(bool_to_u32(event.is_initial_reference)), static_cast<unsigned long>(bool_to_u32(event.is_mission_seed)), static_cast<unsigned>(event.source_pose_id), static_cast<unsigned>(event.source_branch_id), static_cast<unsigned long>(event.activation_time_ms), static_cast<unsigned>(event.reference_confidence), static_cast<unsigned long>(event.reference_sample_id), static_cast<long long>(event.local_reference_x_um), static_cast<long long>(event.local_reference_y_um), static_cast<long>(event.local_reference_heading_urad), static_cast<long long>(event.global_reference_x_um), static_cast<long long>(event.global_reference_y_um), static_cast<long>(event.global_reference_heading_urad), static_cast<long>(event.rotation_urad), static_cast<long>(event.matrix_cos_ppm), static_cast<long>(event.matrix_sin_ppm), static_cast<long>(event.matrix_cos_ppm), static_cast<long>(-event.matrix_sin_ppm), static_cast<long>(event.matrix_sin_ppm), static_cast<long>(event.matrix_cos_ppm));

        if ((length <= 0) || (static_cast<std::size_t>(length) >= sizeof(response)))
        {
            return false;
        }

        return handler_helpers::write_response_text(response);
    }

    bool handle_get_anchor_change_debug()
    {
        if (middleware_parse_helpers::read_end(debug_handler_helpers::timeout_us) == false)
        {
            return debug_handler_helpers::write_bad_format();
        }

        const local_to_global_transform::anchor_event_snapshot event = local_to_global_transform::read_anchor_event();

        if (event.valid == false)
        {
            return true;
        }

        if (event.event_id == last_sent_anchor_change_id)
        {
            return true;
        }

        last_sent_anchor_change_id = event.event_id;

        const std::int64_t rotated_local_x_um =
            ((event.local_reference_x_um * static_cast<std::int64_t>(event.matrix_cos_ppm)) -
             (event.local_reference_y_um * static_cast<std::int64_t>(event.matrix_sin_ppm))) / 1000000LL;
        const std::int64_t rotated_local_y_um =
            ((event.local_reference_x_um * static_cast<std::int64_t>(event.matrix_sin_ppm)) +
             (event.local_reference_y_um * static_cast<std::int64_t>(event.matrix_cos_ppm))) / 1000000LL;
        const std::int64_t offset_x_um = event.global_reference_x_um - rotated_local_x_um;
        const std::int64_t offset_y_um = event.global_reference_y_um - rotated_local_y_um;

        char response[420] = {};
        const int length = std::snprintf(
            response,
            sizeof(response),
            "anchor_change event_id %lu type %u pose_id %u branch_id %u confidence %u x_um %lld y_um %lld jump_valid %lu jump_x_um %lld jump_y_um %lld jump_heading_urad %ld offset_x_um %lld offset_y_um %lld rotation_urad %ld",
            static_cast<unsigned long>(event.event_id),
            static_cast<unsigned>(event.type),
            static_cast<unsigned>(event.source_pose_id),
            static_cast<unsigned>(event.source_branch_id),
            static_cast<unsigned>(event.reference_confidence),
            static_cast<long long>(event.global_reference_x_um),
            static_cast<long long>(event.global_reference_y_um),
            static_cast<unsigned long>(bool_to_u32(event.has_position_jump)),
            static_cast<long long>(event.position_jump_x_um),
            static_cast<long long>(event.position_jump_y_um),
            static_cast<long>(event.heading_jump_urad),
            static_cast<long long>(offset_x_um),
            static_cast<long long>(offset_y_um),
            static_cast<long>(event.rotation_urad));

        if ((length <= 0) || (static_cast<std::size_t>(length) >= sizeof(response)))
        {
            return false;
        }

        return handler_helpers::write_response_text(response);
    }
}
