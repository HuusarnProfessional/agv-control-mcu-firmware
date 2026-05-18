#include "../debug_handler_declarations.hpp"

#include <Arduino.h>

#include <cstdarg>
#include <cstddef>
#include <cstdint>
#include <cstdio>

#include "../../../middleware_parse_helpers.hpp"
#include "../../handler_helpers.hpp"
#include "../../../../../position_sensorfusion/filtered_global_position/filtered_global_position.hpp"
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

    void append_debug_field(bool &formatted, char *response, std::size_t capacity, std::size_t &offset, const char *format, ...)
    {
        if (formatted == false)
        {
            return;
        }

        va_list args;
        va_start(args, format);
        const int written = std::vsnprintf(response + offset, capacity - offset, format, args);
        va_end(args);

        if (written <= 0)
        {
            formatted = false;
            return;
        }

        const std::size_t written_size = static_cast<std::size_t>(written);

        if ((offset + written_size) >= capacity)
        {
            formatted = false;
            return;
        }

        offset += written_size;
    }
}

namespace debug_handlers
{
    bool handle_get_filtered_global_position_debug()
    {
        if (middleware_parse_helpers::read_end(debug_handler_helpers::timeout_us) == false)
        {
            return debug_handler_helpers::write_bad_format();
        }

        const std::uint32_t now_ms = static_cast<std::uint32_t>(millis());
        const filtered_global_position::output_snapshot state = filtered_global_position::read_output(now_ms);

        char response[2300] = {};
        std::size_t offset = 0U;
        bool formatted = true;

        append_debug_field(formatted, response, sizeof(response), offset, "filtered_global_position_debug");
        append_debug_field(formatted, response, sizeof(response), offset, " has_position %lu", static_cast<unsigned long>(bool_to_u32(state.has_position)));
        append_debug_field(formatted, response, sizeof(response), offset, " x_um %lld", static_cast<long long>(state.x_um));
        append_debug_field(formatted, response, sizeof(response), offset, " y_um %lld", static_cast<long long>(state.y_um));
        append_debug_field(formatted, response, sizeof(response), offset, " z_um %lld", static_cast<long long>(state.z_um));
        append_debug_field(formatted, response, sizeof(response), offset, " confidence_position %u", static_cast<unsigned>(state.confidence_position));
        append_debug_field(formatted, response, sizeof(response), offset, " has_heading %lu", static_cast<unsigned long>(bool_to_u32(state.has_heading)));
        append_debug_field(formatted, response, sizeof(response), offset, " heading_urad %ld", static_cast<long>(state.heading_urad));
        append_debug_field(formatted, response, sizeof(response), offset, " confidence_heading %u", static_cast<unsigned>(state.confidence_heading));
        append_debug_field(formatted, response, sizeof(response), offset, " is_new_sample %lu", static_cast<unsigned long>(bool_to_u32(state.is_new_sample)));
        append_debug_field(formatted, response, sizeof(response), offset, " accepted %lu", static_cast<unsigned long>(bool_to_u32(state.accepted)));
        append_debug_field(formatted, response, sizeof(response), offset, " rejected %lu", static_cast<unsigned long>(bool_to_u32(state.rejected)));
        append_debug_field(formatted, response, sizeof(response), offset, " raw_confidence_position %u", static_cast<unsigned>(state.raw_confidence_position));
        append_debug_field(formatted, response, sizeof(response), offset, " history_confidence %u", static_cast<unsigned>(state.history_confidence));
        append_debug_field(formatted, response, sizeof(response), offset, " accepted_sample_count %u", static_cast<unsigned>(state.accepted_sample_count));
        append_debug_field(formatted, response, sizeof(response), offset, " heading_sample_count %u", static_cast<unsigned>(state.heading_sample_count));
        append_debug_field(formatted, response, sizeof(response), offset, " heading_distance_um %lld", static_cast<long long>(state.heading_distance_um));
        append_debug_field(formatted, response, sizeof(response), offset, " heading_mode %u", static_cast<unsigned>(state.heading_mode));
        append_debug_field(formatted, response, sizeof(response), offset, " heading_reference_time_ms %lu", static_cast<unsigned long>(state.heading_reference_time_ms));
        append_debug_field(formatted, response, sizeof(response), offset, " heading_reference_sample_id %lu", static_cast<unsigned long>(state.heading_reference_sample_id));
        append_debug_field(formatted, response, sizeof(response), offset, " heading_estimated_delay_ms %lu", static_cast<unsigned long>(state.heading_estimated_delay_ms));
        append_debug_field(formatted, response, sizeof(response), offset, " position_reference_time_ms %lu", static_cast<unsigned long>(state.position_reference_time_ms));
        append_debug_field(formatted, response, sizeof(response), offset, " position_reference_sample_id %lu", static_cast<unsigned long>(state.position_reference_sample_id));
        append_debug_field(formatted, response, sizeof(response), offset, " position_reference_pose_id %u", static_cast<unsigned>(state.position_reference_pose_id));
        append_debug_field(formatted, response, sizeof(response), offset, " position_reference_branch_id %u", static_cast<unsigned>(state.position_reference_branch_id));
        append_debug_field(formatted, response, sizeof(response), offset, " position_reference_x_um %lld", static_cast<long long>(state.position_reference_x_um));
        append_debug_field(formatted, response, sizeof(response), offset, " position_reference_y_um %lld", static_cast<long long>(state.position_reference_y_um));
        append_debug_field(formatted, response, sizeof(response), offset, " position_reference_z_um %lld", static_cast<long long>(state.position_reference_z_um));
        append_debug_field(formatted, response, sizeof(response), offset, " position_anchor_sample_count %u", static_cast<unsigned>(state.position_anchor_sample_count));
        append_debug_field(formatted, response, sizeof(response), offset, " position_anchor_median_residual_um %lu", static_cast<unsigned long>(state.position_anchor_median_residual_um));
        append_debug_field(formatted, response, sizeof(response), offset, " position_anchor_window_age_ms %lu", static_cast<unsigned long>(state.position_anchor_window_age_ms));
        append_debug_field(formatted, response, sizeof(response), offset, " heading_reference_pose_id %u", static_cast<unsigned>(state.heading_reference_pose_id));
        append_debug_field(formatted, response, sizeof(response), offset, " heading_reference_branch_id %u", static_cast<unsigned>(state.heading_reference_branch_id));
        append_debug_field(formatted, response, sizeof(response), offset, " heading_reference_x_um %lld", static_cast<long long>(state.heading_reference_x_um));
        append_debug_field(formatted, response, sizeof(response), offset, " heading_reference_y_um %lld", static_cast<long long>(state.heading_reference_y_um));
        append_debug_field(formatted, response, sizeof(response), offset, " heading_reference_z_um %lld", static_cast<long long>(state.heading_reference_z_um));
        append_debug_field(formatted, response, sizeof(response), offset, " heading_fit_residual_um %lu", static_cast<unsigned long>(state.heading_fit_residual_um));
        append_debug_field(formatted, response, sizeof(response), offset, " candidate_anchor_position_confidence %u", static_cast<unsigned>(state.candidate_anchor_position_confidence));
        append_debug_field(formatted, response, sizeof(response), offset, " candidate_anchor_heading_confidence %u", static_cast<unsigned>(state.candidate_anchor_heading_confidence));
        append_debug_field(formatted, response, sizeof(response), offset, " candidate_anchor_adjusted_heading_confidence %u", static_cast<unsigned>(state.candidate_anchor_adjusted_heading_confidence));
        append_debug_field(formatted, response, sizeof(response), offset, " candidate_anchor_confidence %u", static_cast<unsigned>(state.candidate_anchor_confidence));
        append_debug_field(formatted, response, sizeof(response), offset, " candidate_position_anchor_confidence %u", static_cast<unsigned>(state.candidate_position_anchor_confidence));
        append_debug_field(formatted, response, sizeof(response), offset, " huber_pca_used_sample_count %u", static_cast<unsigned>(state.huber_pca_used_sample_count));
        append_debug_field(formatted, response, sizeof(response), offset, " huber_pca_median_residual_um %lu", static_cast<unsigned long>(state.huber_pca_median_residual_um));
        append_debug_field(formatted, response, sizeof(response), offset, " huber_pca_max_residual_um %lu", static_cast<unsigned long>(state.huber_pca_max_residual_um));
        append_debug_field(formatted, response, sizeof(response), offset, " huber_pca_movement_distance_um %lu", static_cast<unsigned long>(state.huber_pca_movement_distance_um));
        append_debug_field(formatted, response, sizeof(response), offset, " huber_pca_window_age_ms %lu", static_cast<unsigned long>(state.huber_pca_window_age_ms));
        append_debug_field(formatted, response, sizeof(response), offset, " chord_used_sample_count %u", static_cast<unsigned>(state.chord_used_sample_count));
        append_debug_field(formatted, response, sizeof(response), offset, " chord_distance_um %lld", static_cast<long long>(state.chord_distance_um));
        append_debug_field(formatted, response, sizeof(response), offset, " chord_max_line_error_um %lld", static_cast<long long>(state.chord_max_line_error_um));
        append_debug_field(formatted, response, sizeof(response), offset, " chord_window_age_ms %lu", static_cast<unsigned long>(state.chord_window_age_ms));
        append_debug_field(formatted, response, sizeof(response), offset, " sample_id %lu", static_cast<unsigned long>(state.sample_id));
        append_debug_field(formatted, response, sizeof(response), offset, " request_id %lu", static_cast<unsigned long>(state.request_id));
        append_debug_field(formatted, response, sizeof(response), offset, " received_time_ms %lu", static_cast<unsigned long>(state.received_time_ms));

        if (formatted == false)
        {
            return false;
        }

        return handler_helpers::write_response_text(response);
    }
}
