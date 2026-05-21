#pragma once

#include <cstdint>

#include "../filtered_global.hpp"

namespace adaptive_low_pass
{
    filtered_global::sample update_position(const filtered_global::sample &previous_sample, const filtered_global::sample &raw_sample, std::uint16_t filtered_confidence);
}
