#include "debug_stream_control_payload.hpp"

#include "../../motion_mcu_routes.hpp"
#include "../outgoing_payload_definition.hpp"

namespace debug_stream_control_payload
{
    bool send(std::uint8_t target_payload_id, bool is_enabled)
    {
        outgoing_payload_definition::payload_buffer payload = {};
        payload.payload_id = static_cast<std::uint8_t>(motion_mcu_routes::outgoing_payload_id::debug_stream_control);
        payload.payload_length = 2U;
        payload.payload_data[0] = target_payload_id;
        payload.payload_data[1] = is_enabled ? 1U : 0U;
        return outgoing_payload_definition::send_payload(payload);
    }
}
