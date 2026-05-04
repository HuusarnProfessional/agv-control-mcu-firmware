#include "../incoming_request_handler_declarations.hpp"

#include "../../../middleware_parse_helpers.hpp"
#include "../../handler_helpers.hpp"
#include "../../../../../motion_mcu_communication/outgoing_payloads/service/lock_safe_guard_payload.hpp"
#include "../../../../../motion_mcu_communication/outgoing_payloads/service/unlock_safe_guard_payload.hpp"

namespace incoming_request_handlers
{
    bool handle_set_armed()
    {
        bool is_armed = false;
        const bool parsed_ok = middleware_parse_helpers::read_bool_and_end(is_armed, 50000u);

        if (parsed_ok == false)
        {
            return false;
        }

        bool command_sent = false;

        if (is_armed == true)
        {
            command_sent = unlock_safe_guard_payload::send();
        }
        else
        {
            command_sent = lock_safe_guard_payload::send();
        }

        if (command_sent == false)
        {
            const bool fail_response_written = handler_helpers::write_response("rsp:fail(0)");

            if (fail_response_written == false)
            {
                return false;
            }

            return false;
        }

        const bool ok_response_written = handler_helpers::write_response("rsp:ok()");

        if (ok_response_written == false)
        {
            return false;
        }

        return true;
    }
}
