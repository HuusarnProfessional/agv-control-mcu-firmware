#include "../debug_handler_declarations.hpp"

#include <cstddef>
#include <cstdio>

#include "../../../middleware_parse_helpers.hpp"
#include "../../handler_helpers.hpp"
#include "../../../../../position_sensorfusion/filtered_global_offset_fusion/filtered_global_offset_fusion.hpp"
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
    bool handle_get_filtered_global_offset_fusion_debug()
    {
        if (middleware_parse_helpers::read_end(debug_handler_helpers::timeout_us) == false)
        {
            return debug_handler_helpers::write_bad_format();
        }

        const filtered_global_offset_fusion::output_snapshot state = filtered_global_offset_fusion::read_output();

        char response[384] = {};
        const int length = std::snprintf(response, sizeof(response), "filtered_global_offset_fusion_debug has_pose %lu x_um %lld y_um %lld heading_urad %ld confidence_position %u confidence_heading %u pose_id %u branch_id %u has_offset %lu has_heading_offset %lu", static_cast<unsigned long>(bool_to_u32(state.has_pose)), static_cast<long long>(state.x_um), static_cast<long long>(state.y_um), static_cast<long>(state.heading_urad), static_cast<unsigned>(state.confidence_position), static_cast<unsigned>(state.confidence_heading), static_cast<unsigned>(state.pose_id), static_cast<unsigned>(state.branch_id), static_cast<unsigned long>(bool_to_u32(state.has_offset)), static_cast<unsigned long>(bool_to_u32(state.has_heading_offset)));

        if ((length <= 0) || (static_cast<std::size_t>(length) >= sizeof(response)))
        {
            return false;
        }

        return handler_helpers::write_response_text(response);
    }
}
