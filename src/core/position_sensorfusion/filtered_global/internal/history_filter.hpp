#pragma once

#include <cstdint>

#include "history_buffer.hpp"

namespace history_filter
{
    std::uint16_t calculate_history_confidence(const filtered_global_history::state &history, const filtered_global::sample &raw_sample);
}
