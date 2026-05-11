#include "alignment_pending_request.hpp"

namespace
{
    constexpr std::uint32_t settling_time_ms = 500U;

    std::uint32_t get_age_ms(std::uint32_t now_ms, std::uint32_t start_time_ms)
    {
        if (now_ms < start_time_ms)
        {
            return 0U;
        }

        return now_ms - start_time_ms;
    }
}

namespace alignment_pending_request
{
    void init(pending_state &state)
    {
        state = {};
    }

    bool branch_has_arrived(const pending_state &state, std::uint8_t current_branch_id)
    {
        if (state.pending == false)
        {
            return false;
        }

        if (current_branch_id != state.source_branch_id)
        {
            return true;
        }

        return false;
    }

    void mark_branch_arrived(pending_state &state, std::uint32_t now_ms)
    {
        state.pending = false;
        state.settling = true;
        state.settling_start_time_ms = now_ms;
    }

    void update_settling(pending_state &state, std::uint32_t now_ms)
    {
        if (state.settling == false)
        {
            return;
        }

        const std::uint32_t age_ms = get_age_ms(now_ms, state.settling_start_time_ms);

        if (age_ms >= settling_time_ms)
        {
            state.settling = false;
        }
    }

    void start(pending_state &state, std::uint8_t pose_id, std::uint8_t branch_id, std::uint32_t global_sample_id, std::uint16_t anchor_score, std::uint32_t now_ms)
    {
        state.pending = true;
        state.source_pose_id = pose_id;
        state.source_branch_id = branch_id;
        state.request_time_ms = now_ms;
        state.global_sample_id = global_sample_id;
        state.anchor_score = anchor_score;

        state.has_last_request = true;
        state.last_request_time_ms = now_ms;
        state.last_request_global_sample_id = global_sample_id;
        state.last_request_score = anchor_score;
    }
}