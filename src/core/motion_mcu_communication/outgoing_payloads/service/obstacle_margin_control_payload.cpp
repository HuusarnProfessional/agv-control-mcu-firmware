#include "obstacle_margin_control_payload.hpp"

#include "../../motion_mcu_routes.hpp"
#include "../../payload_helper_functions.hpp"
#include "../outgoing_payload_definition.hpp"

namespace obstacle_margin_control_payload
{
    bool send(std::uint16_t margin_mm)
    {
        outgoing_payload_definition::payload_buffer payload = {};
        payload.payload_id = static_cast<std::uint8_t>(motion_mcu_routes::outgoing_payload_id::obstacle_margin_control);
        payload.payload_length = 2U;

        if (payload_helper_functions::write_u16_le(payload.payload_data, sizeof(payload.payload_data), 0U, margin_mm) == false)
        {
            return false;
        }

        return outgoing_payload_definition::send_payload(payload);
    }
}
