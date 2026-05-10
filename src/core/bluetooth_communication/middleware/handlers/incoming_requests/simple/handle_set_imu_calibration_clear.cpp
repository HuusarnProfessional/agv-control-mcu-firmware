#include "../incoming_request_handler_declarations.hpp"

#include "../../../middleware_parse_helpers.hpp"
#include "../../handler_helpers.hpp"
#include "../../../../../motion_mcu_communication/outgoing_payloads/service/clear_imu_calibration_payload.hpp"

namespace incoming_request_handlers
{
    bool handle_set_imu_calibration_clear()
    {
        const bool parsed_ok = middleware_parse_helpers::read_end(50000u);

        if (parsed_ok == false)
        {
            return false;
        }

        if (clear_imu_calibration_payload::send() == false)
        {
            return handler_helpers::write_response("rsp:fail(0)");
        }

        return handler_helpers::write_response("rsp:ok()");
    }
}
