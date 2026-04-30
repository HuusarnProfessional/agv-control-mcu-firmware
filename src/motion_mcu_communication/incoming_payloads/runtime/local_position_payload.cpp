#include "local_position_payload.hpp"

#include "../../payload_helper_functions.hpp"
#include "../../motion_mcu_runtime.hpp"

namespace local_position_payload
{
    void handle(const std::uint8_t *payload_data, std::uint8_t payload_length)
    {
        motion_mcu_runtime::local_position_state state = {};

        bool has_x = payload_helper_functions::read_i16_le(payload_data, payload_length, 0U, state.x_mm);
        bool has_y = payload_helper_functions::read_i16_le(payload_data, payload_length, 2U, state.y_mm);
        bool has_heading = payload_helper_functions::read_i16_le(payload_data, payload_length, 4U, state.heading_mrad);

        if ((has_x == true) && (has_y == true) && (has_heading == true))
        {
            state.is_valid = true;
            motion_mcu_runtime::set_local_position(state);
        }
    }
}
