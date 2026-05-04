#include "trailer_status_payload.hpp"

#include "../../motion_mcu_routes.hpp"
#include "../../payload_helper_functions.hpp"
#include "../outgoing_payload_definition.hpp"

namespace trailer_status_payload
{
    bool send(bool has_trailer)
    {
        outgoing_payload_definition::payload_buffer payload = {};
        payload.payload_id = static_cast<std::uint8_t>(motion_mcu_routes::outgoing_payload_id::trailer_status);
        payload.payload_length = 1U;

        if (payload_helper_functions::write_bool(payload.payload_data, sizeof(payload.payload_data), 0U, has_trailer) == false)
        {
            return false;
        }

        return outgoing_payload_definition::send_payload(payload);
    }
}
