#pragma once

#include <cstdint>

namespace debug_stream_control_payload
{
    bool send(std::uint8_t target_payload_id, bool is_enabled);
}
