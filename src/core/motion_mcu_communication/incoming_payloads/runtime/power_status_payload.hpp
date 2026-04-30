#pragma once

#include <cstdint>

namespace power_status_payload
{
    void handle(const std::uint8_t *payload_data, std::uint8_t payload_length);
}
