#include "alignment_request_gate.hpp"

namespace
{
    constexpr std::uint16_t normal_request_margin = 100U;
    constexpr std::uint16_t pending_upgrade_margin = 150U;
    constexpr std::uint16_t interval_override_margin = 300U;

    constexpr std::uint32_t minimum_request_interval_ms = 1000U;

    std::uint32_t get_age_ms(std::uint32_t now_ms, std::uint32_t start_time_ms)
    {
        if (now_ms < start_time_ms)
        {
            return 0U;
        }

        return now_ms - start_time_ms;
    }

    bool score_is_better(std::uint16_t new_score, std::uint16_t old_score, std::uint16_t margin)
    {
        const std::uint32_t required_score = static_cast<std::uint32_t>(old_score) + static_cast<std::uint32_t>(margin);

        if (static_cast<std::uint32_t>(new_score) > required_score)
        {
            return true;
        }

        return false;
    }

    bool normal_interval_has_passed(const alignment_pending_request::pending_state &state, std::uint32_t now_ms)
    {
        if (state.has_last_request == false)
        {
            return true;
        }

        const std::uint32_t age_ms = get_age_ms(now_ms, state.last_request_time_ms);

        if (age_ms >= minimum_request_interval_ms)
        {
            return true;
        }

        return false;
    }
}

namespace alignment_request_gate
{
    request_decision evaluate(const alignment_pending_request::pending_state &state, std::uint16_t anchor_score, std::uint16_t corrected_local_score, std::uint32_t global_sample_id, std::uint32_t now_ms)
    {
        request_decision decision = {};

        if (anchor_score == 0U)
        {
            return decision;
        }

        if (state.has_last_request == true)
        {
            if (global_sample_id == state.last_request_global_sample_id)
            {
                return decision;
            }
        }

        if (score_is_better(anchor_score, corrected_local_score, normal_request_margin) == false)
        {
            return decision;
        }

        if (state.pending == true)
        {
            if (score_is_better(anchor_score, state.anchor_score, pending_upgrade_margin) == true)
            {
                decision.should_request = true;
                decision.is_upgrade = true;
            }

            return decision;
        }

        if (state.settling == true)
        {
            return decision;
        }

        if (normal_interval_has_passed(state, now_ms) == true)
        {
            decision.should_request = true;
            decision.is_upgrade = false;

            return decision;
        }

        if (score_is_better(anchor_score, state.last_request_score, interval_override_margin) == true)
        {
            decision.should_request = true;
            decision.is_upgrade = true;
        }

        return decision;
    }
}