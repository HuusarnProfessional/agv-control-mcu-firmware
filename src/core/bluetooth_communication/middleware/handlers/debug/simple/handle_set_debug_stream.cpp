#include "../debug_handler_declarations.hpp"

#include <cstddef>
#include <cstdint>

#include "../../../middleware_parse_helpers.hpp"
#include "../../handler_helpers.hpp"
#include "../../../../../motion_mcu_communication/outgoing_payloads/service/debug_stream_control_payload.hpp"
#include "../../../../../motion_mcu_communication/state/debug/debug_state.hpp"

namespace
{
    bool strings_equal(const char *left, const char *right)
    {
        if ((left == nullptr) || (right == nullptr))
        {
            return false;
        }

        std::size_t index = 0U;

        while ((left[index] != '\0') && (right[index] != '\0'))
        {
            if (left[index] != right[index])
            {
                return false;
            }

            ++index;
        }

        return (left[index] == '\0') && (right[index] == '\0');
    }

    std::uint8_t stream_name_to_payload_id(const char *stream_name)
    {
        if (strings_equal(stream_name, "encoder_debug") == true)
        {
            return 0x11U;
        }

        if (strings_equal(stream_name, "imu_debug") == true)
        {
            return 0x12U;
        }

        if (strings_equal(stream_name, "obstacle_debug") == true)
        {
            return 0x13U;
        }

        if (strings_equal(stream_name, "voltage_debug") == true)
        {
            return 0x14U;
        }

        if (strings_equal(stream_name, "motion_debug") == true)
        {
            return 0x1DU;
        }

        if (strings_equal(stream_name, "local_position_model_debug") == true)
        {
            return 0x1EU;
        }

        return 0U;
    }
}

namespace debug_handlers
{
    bool handle_set_debug_stream()
    {
        char stream_name[48] = {};
        bool ended = false;

        if (middleware_parse_helpers::read_until_comma_or_end(stream_name, sizeof(stream_name), ended, 50000U) == false)
        {
            return false;
        }

        if (ended == true)
        {
            return handler_helpers::write_response_text("err bad_format");
        }

        bool is_enabled = false;

        if (middleware_parse_helpers::read_bool_and_end(is_enabled, 50000U) == false)
        {
            return handler_helpers::write_response_text("err bad_format");
        }

        const std::uint8_t payload_id = stream_name_to_payload_id(stream_name);

        if (payload_id == 0U)
        {
            return handler_helpers::write_response_text("err unknown_stream");
        }

        if (debug_stream_control_payload::send(payload_id, is_enabled) == false)
        {
            return handler_helpers::write_response_text("err send_failed");
        }

        motion_mcu_debug_state::set_stream_enabled(payload_id, is_enabled);
        return handler_helpers::write_response_text("ok");
    }
}
