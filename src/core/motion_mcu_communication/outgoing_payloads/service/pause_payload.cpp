#include "pause_payload.hpp"

#include "../../motion_mcu_routes.hpp"
#include "../../payload_helper_functions.hpp"
#include "../outgoing_payload_definition.hpp"

namespace pause_payload
{
    bool send(std::uint32_t duration_ms)
    {
        outgoing_payload_definition::payload_buffer payload = {};
        payload.payload_id = static_cast<std::uint8_t>(motion_mcu_routes::outgoing_payload_id::pause);
        payload.payload_length = 4U;

        if (payload_helper_functions::write_u32_le(payload.payload_data, sizeof(payload.payload_data), 0U, duration_ms) == false)
        {
            return false;
        }

        return outgoing_payload_definition::send_payload(payload);
    }
}
