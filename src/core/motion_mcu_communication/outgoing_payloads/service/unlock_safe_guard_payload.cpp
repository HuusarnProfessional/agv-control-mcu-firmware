#include "unlock_safe_guard_payload.hpp"

#include "../../motion_mcu_routes.hpp"
#include "../outgoing_payload_definition.hpp"

namespace unlock_safe_guard_payload
{
    bool send()
    {
        outgoing_payload_definition::payload_buffer payload = {};
        payload.payload_id = static_cast<std::uint8_t>(motion_mcu_routes::outgoing_payload_id::unlock_safe_guard);
        payload.payload_length = 0U;
        return outgoing_payload_definition::send_payload(payload);
    }
}
