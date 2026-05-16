#pragma once

#include <cstdint>

#include "../../motion_mcu_communication/state/incoming/incoming_state.hpp"

namespace filtered_global_position
{
    enum class filtered_global_heading_mode : std::uint8_t
    {
        chord = 0U,
        huber_pca = 1U
    };

    struct output_snapshot
    {
        bool has_position = false;
        std::int64_t x_um = 0;
        std::int64_t y_um = 0;
        std::int64_t z_um = 0;
        std::uint16_t confidence_position = 0U;

        bool has_heading = false;
        std::int32_t heading_urad = 0;
        std::uint16_t confidence_heading = 0U;

        bool is_new_sample = false;
        bool accepted = false;
        bool rejected = false;

        std::uint16_t raw_confidence_position = 0U;
        std::uint16_t history_confidence = 0U;
        std::uint8_t accepted_sample_count = 0U;
        std::uint8_t heading_sample_count = 0U;
        std::int64_t heading_distance_um = 0;
        std::uint8_t heading_mode = 0U;
        std::uint32_t heading_reference_time_ms = 0U;
        std::uint32_t heading_reference_sample_id = 0U;
        std::uint32_t heading_estimated_delay_ms = 0U;
        std::uint16_t heading_reference_pose_id = 0U;
        std::uint8_t heading_reference_branch_id = 0U;
        std::int64_t heading_reference_x_um = 0;
        std::int64_t heading_reference_y_um = 0;
        std::int64_t heading_reference_z_um = 0;
        std::uint32_t heading_fit_residual_um = 0U;
        std::uint8_t huber_pca_used_sample_count = 0U;
        std::uint32_t huber_pca_median_residual_um = 0U;
        std::uint32_t huber_pca_max_residual_um = 0U;
        std::uint32_t huber_pca_movement_distance_um = 0U;
        std::uint32_t huber_pca_window_age_ms = 0U;
        std::uint8_t chord_used_sample_count = 0U;
        std::int64_t chord_distance_um = 0;
        std::int64_t chord_max_line_error_um = 0;
        std::uint32_t chord_window_age_ms = 0U;

        std::uint32_t sample_id = 0U;
        std::uint32_t request_id = 0U;
        std::uint32_t received_time_ms = 0U;
    };

    void init();

    output_snapshot update(std::uint32_t now_ms, const motion_mcu_incoming_state::local_position_state &local_position);

    output_snapshot read_output(std::uint32_t now_ms);
}
