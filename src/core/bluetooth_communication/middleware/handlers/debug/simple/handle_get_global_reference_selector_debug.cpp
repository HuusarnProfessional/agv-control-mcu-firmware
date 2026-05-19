#include "../debug_handler_declarations.hpp"

#include <cstddef>
#include <cstdio>

#include "../../../middleware_parse_helpers.hpp"
#include "../../handler_helpers.hpp"
#include "../../../../../position_sensorfusion/global_reference_selector/global_reference_selector.hpp"
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
    bool handle_get_global_reference_selector_debug()
    {
        if (middleware_parse_helpers::read_end(debug_handler_helpers::timeout_us) == false)
        {
            return debug_handler_helpers::write_bad_format();
        }

        const global_reference_selector::output_snapshot state = global_reference_selector::read_output();

        char response[1200] = {};
        const int length = std::snprintf(response, sizeof(response), "global_reference_selector_debug pending %lu settling %lu pending_pose_id %u pending_branch_id %u pending_global_sample_id %lu local_position_confidence %u local_heading_confidence %u local_reference_confidence %u candidate_anchor_position_confidence %u candidate_anchor_heading_confidence %u candidate_anchor_adjusted_heading_confidence %u candidate_anchor_confidence %u candidate_position_anchor_confidence %u candidate_anchor_type %u candidate_anchor_heading_delta_urad %ld candidate_anchor_heading_consistent %lu required_anchor_confidence %u request_reason %u request %lu request_type %u request_pose_id %u request_branch_id %u request_confidence %u activation %lu activation_type %u initial %lu activation_pose_id %u activation_branch_id %u activation_confidence %u activation_sample_id %lu", static_cast<unsigned long>(bool_to_u32(state.pending)), static_cast<unsigned long>(bool_to_u32(state.settling)), static_cast<unsigned>(state.pending_pose_id), static_cast<unsigned>(state.pending_branch_id), static_cast<unsigned long>(state.pending_global_sample_id), static_cast<unsigned>(state.local_position_confidence), static_cast<unsigned>(state.local_heading_confidence), static_cast<unsigned>(state.local_reference_confidence), static_cast<unsigned>(state.candidate_anchor_position_confidence), static_cast<unsigned>(state.candidate_anchor_heading_confidence), static_cast<unsigned>(state.candidate_anchor_adjusted_heading_confidence), static_cast<unsigned>(state.candidate_anchor_confidence), static_cast<unsigned>(state.candidate_position_anchor_confidence), static_cast<unsigned>(state.candidate_anchor_type), static_cast<long>(state.candidate_anchor_heading_delta_urad), static_cast<unsigned long>(bool_to_u32(state.candidate_anchor_heading_consistent)), static_cast<unsigned>(state.required_anchor_confidence), static_cast<unsigned>(state.request_reason), static_cast<unsigned long>(bool_to_u32(state.request.has_request)), static_cast<unsigned>(state.request.type), static_cast<unsigned>(state.request.pose_id), static_cast<unsigned>(state.request.branch_id), static_cast<unsigned>(state.request.reference_confidence), static_cast<unsigned long>(bool_to_u32(state.activation.has_activation)), static_cast<unsigned>(state.activation.type), static_cast<unsigned long>(bool_to_u32(state.activation.is_initial_reference)), static_cast<unsigned>(state.activation.source_pose_id), static_cast<unsigned>(state.activation.source_branch_id), static_cast<unsigned>(state.activation.reference_confidence), static_cast<unsigned long>(state.activation.global_reference.sample_id));

        if ((length <= 0) || (static_cast<std::size_t>(length) >= sizeof(response)))
        {
            return false;
        }

        return handler_helpers::write_response_text(response);
    }
}
