#pragma once

#include <cstdint>

#include "../filtered_global.hpp"

namespace bootstrap_filter
{
    bool build_initial_sample(const filtered_global::sample *samples, std::uint8_t sample_count, filtered_global::sample &sample_out, std::uint16_t &history_confidence_out);
}
