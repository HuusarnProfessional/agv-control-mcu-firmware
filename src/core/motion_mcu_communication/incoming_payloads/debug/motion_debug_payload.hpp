#pragma once

#include <cstdint>

namespace motion_debug_payload
{
    void handle(const std::uint8_t *payload_data, std::uint8_t payload_length);
}
