#include "../debug_handler_declarations.hpp"

#include "../../../middleware_parse_helpers.hpp"
#include "../../handler_helpers.hpp"
#include "../../../../debug_capture/position_capture_test_pipeline.hpp"
#include "debug_handler_helpers.hpp"

namespace debug_handlers
{
    bool handle_run_position_capture_test()
    {
        if (middleware_parse_helpers::read_end(debug_handler_helpers::timeout_us) == false)
        {
            return debug_handler_helpers::write_bad_format();
        }

        if (position_capture_test_pipeline::request_run() == false)
        {
            return handler_helpers::write_response_text("err position_capture_test_busy");
        }

        return handler_helpers::write_response_text("ok position_capture_test_queued");
    }
}
