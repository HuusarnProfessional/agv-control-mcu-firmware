#include "local_position_payload.hpp"

#include "../../payload_helper_functions.hpp"
#include "../../motion_mcu_runtime.hpp"

namespace local_position_payload
{
    void handle(const std::uint8_t *payload_data, std::uint8_t payload_length)
    {
        motion_mcu_runtime::local_position_state state = {};

        const bool has_pose = payload_helper_functions::read_bool(payload_data, payload_length, 0U, state.has_pose);
        const bool has_x = payload_helper_functions::read_i64_le(payload_data, payload_length, 1U, state.x_um);
        const bool has_y = payload_helper_functions::read_i64_le(payload_data, payload_length, 9U, state.y_um);
        const bool has_heading = payload_helper_functions::read_i32_le(payload_data, payload_length, 17U, state.heading_urad);
        const bool has_confidence_position = payload_helper_functions::read_u16_le(payload_data, payload_length, 21U, state.confidence_position);
        const bool has_confidence_heading = payload_helper_functions::read_u16_le(payload_data, payload_length, 23U, state.confidence_heading);
        const bool has_pose_id = payload_helper_functions::read_u8(payload_data, payload_length, 25U, state.pose_id);
        const bool has_branch_id = payload_helper_functions::read_u8(payload_data, payload_length, 26U, state.branch_id);

        if ((has_pose == true) &&
            (has_x == true) &&
            (has_y == true) &&
            (has_heading == true) &&
            (has_confidence_position == true) &&
            (has_confidence_heading == true) &&
            (has_pose_id == true) &&
            (has_branch_id == true))
        {
            motion_mcu_runtime::set_local_position(state);
        }
    }
}
