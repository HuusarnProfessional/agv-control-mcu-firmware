#pragma once

#include <cstdint>

#include "alignment_pending_request.hpp"

namespace alignment_request_gate
{
    struct request_decision
    {
        bool should_request = false;
        bool is_upgrade = false;
    };

    request_decision evaluate(const alignment_pending_request::pending_state &state, std::uint16_t anchor_score, std::uint16_t corrected_local_score, std::uint32_t global_sample_id, std::uint32_t now_ms);
}