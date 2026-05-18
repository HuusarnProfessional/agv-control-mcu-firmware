#pragma once

#include <cstdint>

#include "../../motion_mcu_communication/state/incoming/incoming_state.hpp"
#include "../filtered_global_position/filtered_global_position.hpp"

namespace global_reference_selector
{
    enum class anchor_type : std::uint8_t
    {
        none = 0U,
        heading_transform = 1U,
        position_only = 2U
    };

    struct current_reference_snapshot
    {
        bool has_reference = false;
        bool has_heading = false;
        std::uint8_t branch_id = 0U;
        std::int32_t heading_urad = 0;
    };

    struct branch_request
    {
        bool has_request = false;
        anchor_type type = anchor_type::none;
        std::uint16_t pose_id = 0U;
        std::uint8_t branch_id = 0U;
        std::uint16_t reference_confidence = 0U;
    };

    struct reference_activation
    {
        bool has_activation = false;
        anchor_type type = anchor_type::none;
        bool is_initial_reference = false;
        bool is_mission_seed = false;
        std::uint16_t source_pose_id = 0U;
        std::uint8_t source_branch_id = 0U;
        std::uint32_t activation_time_ms = 0U;
        std::uint16_t reference_confidence = 0U;
        filtered_global_position::output_snapshot global_reference = {};
    };

    struct output_snapshot
    {
        branch_request request = {};
        reference_activation activation = {};

        bool pending = false;
        bool settling = false;
        std::uint16_t pending_pose_id = 0U;
        std::uint8_t pending_branch_id = 0U;
        std::uint32_t pending_global_sample_id = 0U;
        std::uint16_t local_position_confidence = 0U;
        std::uint16_t local_heading_confidence = 0U;
        std::uint16_t local_reference_confidence = 0U;
        std::uint16_t candidate_anchor_position_confidence = 0U;
        std::uint16_t candidate_anchor_heading_confidence = 0U;
        std::uint16_t candidate_anchor_adjusted_heading_confidence = 0U;
        std::uint16_t candidate_anchor_confidence = 0U;
        std::uint16_t candidate_position_anchor_confidence = 0U;
        anchor_type candidate_anchor_type = anchor_type::none;
        std::int32_t candidate_anchor_heading_delta_urad = 0;
        bool candidate_anchor_heading_consistent = false;
        std::uint16_t required_anchor_confidence = 0U;
        std::uint8_t request_reason = 0U;
    };

    void init();

    void reset_runtime_state();

    output_snapshot update(const motion_mcu_incoming_state::local_position_state &local_position, const filtered_global_position::output_snapshot &global_position, const current_reference_snapshot &current_reference, std::uint32_t now_ms);

    output_snapshot read_output();
}
