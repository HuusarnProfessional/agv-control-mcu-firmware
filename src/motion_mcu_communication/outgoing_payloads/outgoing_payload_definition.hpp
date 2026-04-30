#pragma once

#include <cstdint>

namespace outgoing_payload_definition
{
    struct payload_buffer
    {
        std::uint8_t payload_id;
        std::uint8_t payload_length;
        std::uint8_t payload_data[64U];
    };
}
