#include "drive_forward_payload.hpp"

#include "../../motion_mcu_routes.hpp"
#include "../../payload_helper_functions.hpp"
#include "../outgoing_payload_definition.hpp"

namespace drive_forward_payload
{
    bool send(std::int32_t velocity_mm_s, std::int64_t target_distance_um)
    {
        outgoing_payload_definition::payload_buffer payload = {};
        payload.payload_id = static_cast<std::uint8_t>(motion_mcu_routes::outgoing_payload_id::drive_forward);
        payload.payload_length = 12U;

        if (payload_helper_functions::write_i32_le(payload.payload_data, sizeof(payload.payload_data), 0U, velocity_mm_s) == false)
        {
            return false;
        }

        if (payload_helper_functions::write_i64_le(payload.payload_data, sizeof(payload.payload_data), 4U, target_distance_um) == false)
        {
            return false;
        }

        return outgoing_payload_definition::send_payload(payload);
    }
}
