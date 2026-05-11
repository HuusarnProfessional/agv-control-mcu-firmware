#pragma once

#include <cstdint>

namespace alignment_pending_request
{
    struct pending_state
    {
        bool pending = false;
        std::uint8_t source_pose_id = 0U;
        std::uint8_t source_branch_id = 0U;
        std::uint32_t request_time_ms = 0U;
        std::uint32_t global_sample_id = 0U;
        std::uint16_t anchor_score = 0U;

        bool settling = false;
        std::uint32_t settling_start_time_ms = 0U;

        bool has_last_request = false;
        std::uint32_t last_request_time_ms = 0U;
        std::uint32_t last_request_global_sample_id = 0U;
        std::uint16_t last_request_score = 0U;
    };

    void init(pending_state &state);

    bool branch_has_arrived(const pending_state &state, std::uint8_t current_branch_id);

    void mark_branch_arrived(pending_state &state, std::uint32_t now_ms);

    void update_settling(pending_state &state, std::uint32_t now_ms);

    void start(pending_state &state, std::uint8_t pose_id, std::uint8_t branch_id, std::uint32_t global_sample_id, std::uint16_t anchor_score, std::uint32_t now_ms);
}