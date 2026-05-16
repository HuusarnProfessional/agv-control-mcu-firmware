#include "position_correction_payload.hpp"

#include "../../motion_mcu_routes.hpp"
#include "../../payload_helper_functions.hpp"
#include "../outgoing_payload_definition.hpp"

namespace position_correction_payload
{
    bool send(std::uint16_t pose_id, std::uint8_t branch_id)
    {
        outgoing_payload_definition::payload_buffer payload = {};

        payload.payload_id = static_cast<std::uint8_t>(motion_mcu_routes::outgoing_payload_id::position_correction);
        payload.payload_length = 3U;

        payload_helper_functions::write_u16_le(payload.payload_data, payload.payload_length, 0U, pose_id);
        payload.payload_data[2] = branch_id;

        return outgoing_payload_definition::send_payload(payload);
    }
}
