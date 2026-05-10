#include "../debug_handler_declarations.hpp"

#include <cstddef>
#include <cstdio>

#include "../../../middleware_parse_helpers.hpp"
#include "../../handler_helpers.hpp"
#include "../../../../../motion_mcu_communication/motion_mcu_routes.hpp"
#include "../../../../../motion_mcu_communication/state/debug/debug_state.hpp"
#include "debug_handler_helpers.hpp"

namespace
{
    bool validate_imu_request(motion_mcu_debug_state::imu_debug_state &state_out)
    {
        if (middleware_parse_helpers::read_end(debug_handler_helpers::timeout_us) == false)
        {
            (void)debug_handler_helpers::write_bad_format();
            return false;
        }

        const std::uint8_t payload_id = static_cast<std::uint8_t>(motion_mcu_routes::incoming_payload_id::imu_debug);

        if (motion_mcu_debug_state::is_stream_enabled(payload_id) == false)
        {
            (void)debug_handler_helpers::write_stream_not_active("imu_debug");
            return false;
        }

        state_out = motion_mcu_debug_state::get_imu_debug();

        if (state_out.valid == false)
        {
            (void)debug_handler_helpers::write_missing_data("imu_debug");
            return false;
        }

        return true;
    }
}

namespace debug_handlers
{
    bool handle_get_imu_gyro()
    {
        motion_mcu_debug_state::imu_debug_state state = {};

        if (validate_imu_request(state) == false)
        {
            return false;
        }

        char response[96] = {};

        if (debug_handler_helpers::format_axis3_i32_line("imu_gyro", state.gyro_mdps, response, sizeof(response)) == false)
        {
            return false;
        }

        return handler_helpers::write_response_text(response);
    }

    bool handle_get_imu_accel()
    {
        motion_mcu_debug_state::imu_debug_state state = {};

        if (validate_imu_request(state) == false)
        {
            return false;
        }

        char response[96] = {};

        if (debug_handler_helpers::format_axis3_i32_line("imu_accel", state.accel_mg, response, sizeof(response)) == false)
        {
            return false;
        }

        return handler_helpers::write_response_text(response);
    }

    bool handle_get_imu_mag()
    {
        motion_mcu_debug_state::imu_debug_state state = {};

        if (validate_imu_request(state) == false)
        {
            return false;
        }

        char response[96] = {};

        if (debug_handler_helpers::format_axis3_i32_line("imu_mag", state.mag_mgauss, response, sizeof(response)) == false)
        {
            return false;
        }

        return handler_helpers::write_response_text(response);
    }

    bool handle_get_imu_raw_gyro()
    {
        motion_mcu_debug_state::imu_debug_state state = {};

        if (validate_imu_request(state) == false)
        {
            return false;
        }

        char response[96] = {};

        if (debug_handler_helpers::format_axis3_i16_line("imu_raw_gyro", state.raw_gyro, response, sizeof(response)) == false)
        {
            return false;
        }

        return handler_helpers::write_response_text(response);
    }

    bool handle_get_imu_raw_accel()
    {
        motion_mcu_debug_state::imu_debug_state state = {};

        if (validate_imu_request(state) == false)
        {
            return false;
        }

        char response[96] = {};

        if (debug_handler_helpers::format_axis3_i16_line("imu_raw_accel", state.raw_accel, response, sizeof(response)) == false)
        {
            return false;
        }

        return handler_helpers::write_response_text(response);
    }

    bool handle_get_imu_raw_mag()
    {
        motion_mcu_debug_state::imu_debug_state state = {};

        if (validate_imu_request(state) == false)
        {
            return false;
        }

        char response[96] = {};

        if (debug_handler_helpers::format_axis3_i16_line("imu_raw_mag", state.raw_mag, response, sizeof(response)) == false)
        {
            return false;
        }

        return handler_helpers::write_response_text(response);
    }

    bool handle_get_imu_calibrated_gyro()
    {
        motion_mcu_debug_state::imu_debug_state state = {};

        if (validate_imu_request(state) == false)
        {
            return false;
        }

        char response[96] = {};

        if (debug_handler_helpers::format_axis3_i32_line("imu_calibrated_gyro", state.calibrated_gyro_mdps, response, sizeof(response)) == false)
        {
            return false;
        }

        return handler_helpers::write_response_text(response);
    }

    bool handle_get_imu_calibrated_accel()
    {
        motion_mcu_debug_state::imu_debug_state state = {};

        if (validate_imu_request(state) == false)
        {
            return false;
        }

        char response[96] = {};

        if (debug_handler_helpers::format_axis3_i32_line("imu_calibrated_accel", state.calibrated_accel_mg, response, sizeof(response)) == false)
        {
            return false;
        }

        return handler_helpers::write_response_text(response);
    }

    bool handle_get_imu_status()
    {
        motion_mcu_debug_state::imu_debug_state state = {};

        if (validate_imu_request(state) == false)
        {
            return false;
        }

        char response[128] = {};
        const int length = std::snprintf(
            response,
            sizeof(response),
            "imu_status gyro %u accel %u mag %u has_calibration %u time_ms %lu",
            static_cast<unsigned>(state.gyro_status),
            static_cast<unsigned>(state.accel_status),
            static_cast<unsigned>(state.mag_status),
            state.has_calibration ? 1U : 0U,
            static_cast<unsigned long>(state.time_ms));

        if ((length <= 0) || (static_cast<std::size_t>(length) >= sizeof(response)))
        {
            return false;
        }

        return handler_helpers::write_response_text(response);
    }

    bool handle_get_imu_time_ms()
    {
        motion_mcu_debug_state::imu_debug_state state = {};

        if (validate_imu_request(state) == false)
        {
            return false;
        }

        char response[64] = {};
        const int length = std::snprintf(response, sizeof(response), "imu_time_ms %lu", static_cast<unsigned long>(state.time_ms));

        if ((length <= 0) || (static_cast<std::size_t>(length) >= sizeof(response)))
        {
            return false;
        }

        return handler_helpers::write_response_text(response);
    }

    bool handle_get_imu_has_calibration()
    {
        motion_mcu_debug_state::imu_debug_state state = {};

        if (validate_imu_request(state) == false)
        {
            return false;
        }

        char response[64] = {};
        const int length = std::snprintf(
            response,
            sizeof(response),
            "imu_has_calibration %u",
            state.has_calibration ? 1U : 0U);

        if ((length <= 0) || (static_cast<std::size_t>(length) >= sizeof(response)))
        {
            return false;
        }

        return handler_helpers::write_response_text(response);
    }
}
