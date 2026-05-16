#pragma once

#include <cstdint>

#include "../../motion_mcu_communication/state/incoming/incoming_state.hpp"
#include "../filtered_global_position/filtered_global_position.hpp"

namespace global_reference_selector
{
    struct current_reference_snapshot
    {
        bool has_reference = false;
        std::uint16_t confidence_position = 0U;
        std::uint16_t confidence_heading = 0U;
        std::uint8_t branch_id = 0U;
    };

    struct branch_request
    {
        bool has_request = false;
        std::uint16_t pose_id = 0U;
        std::uint8_t branch_id = 0U;
        std::uint16_t reference_score = 0U;
    };

    struct reference_activation
    {
        bool has_activation = false;
        bool is_initial_reference = false;
        bool is_mission_seed = false;
        std::uint16_t source_pose_id = 0U;
        std::uint8_t source_branch_id = 0U;
        std::uint32_t activation_time_ms = 0U;
        std::uint16_t reference_score = 0U;
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
        std::uint16_t global_reference_score = 0U;
        std::uint16_t current_reference_score = 0U;
    };

    void init();

    void reset_runtime_state();

    output_snapshot update(const motion_mcu_incoming_state::local_position_state &local_position, const filtered_global_position::output_snapshot &global_position, const current_reference_snapshot &current_reference, std::uint32_t now_ms);

    output_snapshot read_output();
}
