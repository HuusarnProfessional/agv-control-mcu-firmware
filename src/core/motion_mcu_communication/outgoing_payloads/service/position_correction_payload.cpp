#include "position_correction_payload.hpp"

#include "../../motion_mcu_routes.hpp"
#include "../outgoing_payload_definition.hpp"

namespace position_correction_payload
{
    bool send(std::uint8_t pose_id, std::uint8_t branch_id)
    {
        outgoing_payload_definition::payload_buffer payload;

        payload.payload_id = static_cast<std::uint8_t>(motion_mcu_routes::outgoing_payload_id::position_correction);
        payload.payload_length = 2;

        payload.payload_data[0] = pose_id;
        payload.payload_data[1] = branch_id;

        (void)payload;

        return false;
    }
}