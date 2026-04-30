#pragma once

#include <cstddef>
#include <cstdint>

namespace incoming_payload_definition
{
    using handler_function = void (*)(const std::uint8_t *payload_data, std::uint8_t payload_length);

    struct route
    {
        std::uint8_t payload_id;
        handler_function handler;
    };
}
