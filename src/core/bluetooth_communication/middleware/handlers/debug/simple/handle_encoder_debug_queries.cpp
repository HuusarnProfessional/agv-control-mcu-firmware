#include "../debug_handler_declarations.hpp"

#include <cstddef>
#include <cstdint>
#include <cstdio>

#include "../../../middleware_parse_helpers.hpp"
#include "../../handler_helpers.hpp"
#include "../../../../../motion_mcu_communication/motion_mcu_routes.hpp"
#include "../../../../../motion_mcu_communication/state/debug/debug_state.hpp"
#include "debug_handler_helpers.hpp"

namespace
{
    bool read_encoder_id(std::uint8_t &encoder_id_out)
    {
        if (middleware_parse_helpers::read_uint8_and_end(encoder_id_out, debug_handler_helpers::timeout_us) == false)
        {
            (void)debug_handler_helpers::write_bad_format();
            return false;
        }

        return true;
    }

    bool validate_encoder_request(motion_mcu_debug_state::encoder_debug_state &state_out, std::uint8_t &encoder_id_out)
    {
        if (read_encoder_id(encoder_id_out) == false)
        {
            return false;
        }

        const std::uint8_t payload_id = static_cast<std::uint8_t>(motion_mcu_routes::incoming_payload_id::encoder_debug);

        if (motion_mcu_debug_state::is_stream_enabled(payload_id) == false)
        {
            (void)debug_handler_helpers::write_stream_not_active("encoder_debug");
            return false;
        }

        state_out = motion_mcu_debug_state::get_encoder_debug();

        if ((state_out.valid == false) || (encoder_id_out >= state_out.count))
        {
            (void)debug_handler_helpers::write_missing_data("encoder");
            return false;
        }

        return true;
    }
}

namespace debug_handlers
{
    bool handle_get_encoder_raw()
    {
        motion_mcu_debug_state::encoder_debug_state state = {};
        std::uint8_t encoder_id = 0U;

        if (validate_encoder_request(state, encoder_id) == false)
        {
            return false;
        }

        char response[64] = {};
        const int length = std::snprintf(
            response,
            sizeof(response),
            "encoder_raw %u %u",
            static_cast<unsigned>(encoder_id),
            static_cast<unsigned>(state.raw[encoder_id]));

        if ((length <= 0) || (static_cast<std::size_t>(length) >= sizeof(response)))
        {
            return false;
        }

        return handler_helpers::write_response_text(response);
    }

    bool handle_get_encoder_deg()
    {
        motion_mcu_debug_state::encoder_debug_state state = {};
        std::uint8_t encoder_id = 0U;

        if (validate_encoder_request(state, encoder_id) == false)
        {
            return false;
        }

        char response[64] = {};
        const int length = std::snprintf(
            response,
            sizeof(response),
            "encoder_deg %u %lu",
            static_cast<unsigned>(encoder_id),
            static_cast<unsigned long>(state.angle_mdeg[encoder_id]));

        if ((length <= 0) || (static_cast<std::size_t>(length) >= sizeof(response)))
        {
            return false;
        }

        return handler_helpers::write_response_text(response);
    }

    bool handle_get_encoder_time()
    {
        motion_mcu_debug_state::encoder_debug_state state = {};
        std::uint8_t encoder_id = 0U;

        if (validate_encoder_request(state, encoder_id) == false)
        {
            return false;
        }

        char response[64] = {};
        const int length = std::snprintf(
            response,
            sizeof(response),
            "encoder_time %u %lu",
            static_cast<unsigned>(encoder_id),
            static_cast<unsigned long>(state.time_ms[encoder_id]));

        if ((length <= 0) || (static_cast<std::size_t>(length) >= sizeof(response)))
        {
            return false;
        }

        return handler_helpers::write_response_text(response);
    }

    bool handle_get_encoder_time_ms()
    {
        return handle_get_encoder_time();
    }

    bool handle_get_encoder_status()
    {
        motion_mcu_debug_state::encoder_debug_state state = {};
        std::uint8_t encoder_id = 0U;

        if (validate_encoder_request(state, encoder_id) == false)
        {
            return false;
        }

        char response[64] = {};
        const int length = std::snprintf(
            response,
            sizeof(response),
            "encoder_status %u %u",
            static_cast<unsigned>(encoder_id),
            static_cast<unsigned>(state.status[encoder_id]));

        if ((length <= 0) || (static_cast<std::size_t>(length) >= sizeof(response)))
        {
            return false;
        }

        return handler_helpers::write_response_text(response);
    }
}
