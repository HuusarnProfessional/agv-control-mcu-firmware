#include "../debug_handler_declarations.hpp"

#include <cstddef>

#include "../../../middleware_parse_helpers.hpp"
#include "../../handler_helpers.hpp"
#include "../../../../../pure_pursuit/pure_pursuit.hpp"
#include "debug_handler_helpers.hpp"

namespace debug_handlers
{
    bool handle_get_pure_pursuit_debug()
    {
        if (middleware_parse_helpers::read_end(debug_handler_helpers::timeout_us) == false)
        {
            return debug_handler_helpers::write_bad_format();
        }

        const pure_pursuit::snapshot state = pure_pursuit::read_snapshot();

        char response[512] = {};
        std::size_t offset = 0U;

        const bool formatted =
            debug_handler_helpers::append_format(response, sizeof(response), offset, "pure_pursuit_debug") &&
            debug_handler_helpers::append_format(response, sizeof(response), offset, " active %u", state.active ? 1U : 0U) &&
            debug_handler_helpers::append_format(response, sizeof(response), offset, " complete %u", state.complete ? 1U : 0U) &&
            debug_handler_helpers::append_format(response, sizeof(response), offset, " success %u", state.success ? 1U : 0U) &&
            debug_handler_helpers::append_format(response, sizeof(response), offset, " has_path %u", state.has_path ? 1U : 0U) &&
            debug_handler_helpers::append_format(response, sizeof(response), offset, " target_valid %u", state.target_valid ? 1U : 0U) &&
            debug_handler_helpers::append_format(response, sizeof(response), offset, " target_forward_ok %u", state.target_forward_ok ? 1U : 0U) &&
            debug_handler_helpers::append_format(response, sizeof(response), offset, " target_curvature_ok %u", state.target_curvature_ok ? 1U : 0U) &&
            debug_handler_helpers::append_format(response, sizeof(response), offset, " part_number %u", static_cast<unsigned>(state.part_number)) &&
            debug_handler_helpers::append_format(response, sizeof(response), offset, " point_count %u", static_cast<unsigned>(state.point_count)) &&
            debug_handler_helpers::append_format(response, sizeof(response), offset, " closest_idx %u", static_cast<unsigned>(state.closest_point_index)) &&
            debug_handler_helpers::append_format(response, sizeof(response), offset, " lookahead_idx %u", static_cast<unsigned>(state.lookahead_point_index)) &&
            debug_handler_helpers::append_format(response, sizeof(response), offset, " linear_mm_s %ld", static_cast<long>(state.linear_velocity_mm_s)) &&
            debug_handler_helpers::append_format(response, sizeof(response), offset, " yaw_rate_mdeg_s %ld", static_cast<long>(state.yaw_rate_mdeg_s)) &&
            debug_handler_helpers::append_format(response, sizeof(response), offset, " target_x_mm %.1f", state.target_x_mm) &&
            debug_handler_helpers::append_format(response, sizeof(response), offset, " target_y_mm %.1f", state.target_y_mm) &&
            debug_handler_helpers::append_format(response, sizeof(response), offset, " target_curvature %.5f", state.target_curvature) &&
            debug_handler_helpers::append_format(response, sizeof(response), offset, " target_distance_mm %.1f", state.target_distance_mm) &&
            debug_handler_helpers::append_format(response, sizeof(response), offset, " forward_mm %.1f", state.forward_mm) &&
            debug_handler_helpers::append_format(response, sizeof(response), offset, " left_mm %.1f", state.left_mm) &&
            debug_handler_helpers::append_format(response, sizeof(response), offset, " heading_error_deg %.2f", state.heading_error_deg);

        if (formatted == false)
        {
            return false;
        }

        return handler_helpers::write_response_text(response);
    }
}
