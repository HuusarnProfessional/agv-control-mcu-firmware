#include "clear_imu_calibration_payload.hpp"

#include "../../motion_mcu_routes.hpp"
#include "../outgoing_payload_definition.hpp"

namespace clear_imu_calibration_payload
{
    bool send()
    {
        outgoing_payload_definition::payload_buffer payload = {};
        payload.payload_id = static_cast<std::uint8_t>(motion_mcu_routes::outgoing_payload_id::clear_imu_calibration);
        payload.payload_length = 0U;
        return outgoing_payload_definition::send_payload(payload);
    }
}
