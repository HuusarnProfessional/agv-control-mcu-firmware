#include "../incoming_request_handler_declarations.hpp"

#include "../../../middleware_parse_helpers.hpp"
#include "../../../../../motion_mcu_communication/outgoing_payloads/service/position_correction_payload.hpp"
#include "../../../../../motion_mcu_communication/state/incoming/incoming_state.hpp"
#include "../../../../../position_sensorfusion/position_sensorfusion.hpp"
#include "../../handler_helpers.hpp"

namespace
{
    constexpr std::uint32_t timeout_us = 50000u;
}

namespace incoming_request_handlers
{
    bool handle_set_position_local_reset()
    {
        if (middleware_parse_helpers::read_end(timeout_us) == false)
        {
            return false;
        }

        const position_sensorfusion::output_snapshot local_position = position_sensorfusion::read_output();

        if (local_position.has_pose == false)
        {
            return handler_helpers::write_response("rsp:fail(1)");
        }

        if (position_correction_payload::send(local_position.pose_id, local_position.branch_id) == false)
        {
            return handler_helpers::write_response("rsp:fail(0)");
        }

        motion_mcu_incoming_state::set_local_position({});
        position_sensorfusion::set_output({});

        return handler_helpers::write_response("rsp:ok()");
    }
}
