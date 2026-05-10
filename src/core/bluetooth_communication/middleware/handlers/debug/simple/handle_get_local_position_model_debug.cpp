#include "../debug_handler_declarations.hpp"

#include <cstddef>

#include "../../../middleware_parse_helpers.hpp"
#include "../../handler_helpers.hpp"
#include "../../../../../motion_mcu_communication/motion_mcu_routes.hpp"
#include "../../../../../motion_mcu_communication/state/debug/debug_state.hpp"
#include "debug_handler_helpers.hpp"

namespace debug_handlers
{
    bool handle_get_local_position_model_debug()
    {
        if (middleware_parse_helpers::read_end(debug_handler_helpers::timeout_us) == false)
        {
            return debug_handler_helpers::write_bad_format();
        }

        const std::uint8_t payload_id = static_cast<std::uint8_t>(motion_mcu_routes::incoming_payload_id::local_position_model_debug);

        if (motion_mcu_debug_state::is_stream_enabled(payload_id) == false)
        {
            return debug_handler_helpers::write_stream_not_active("local_position_model_debug");
        }

        const motion_mcu_debug_state::local_position_model_debug_state state = motion_mcu_debug_state::get_local_position_model_debug();

        if (state.valid == false)
        {
            return debug_handler_helpers::write_missing_data("local_position_model_debug");
        }

        char response[1024] = {};
        std::size_t offset = 0U;

        const bool formatted =
            debug_handler_helpers::append_format(response, sizeof(response), offset, "local_position_model_debug") &&
            debug_handler_helpers::append_format(response, sizeof(response), offset, " has_encoder_motion %u", state.has_encoder_motion ? 1U : 0U) &&
            debug_handler_helpers::append_format(response, sizeof(response), offset, " has_imu_motion %u", state.has_imu_motion ? 1U : 0U) &&
            debug_handler_helpers::append_format(response, sizeof(response), offset, " has_fused_translation %u", state.has_fused_translation ? 1U : 0U) &&
            debug_handler_helpers::append_format(response, sizeof(response), offset, " has_fused_rotation %u", state.has_fused_rotation ? 1U : 0U) &&
            debug_handler_helpers::append_format(response, sizeof(response), offset, " encoder_translation_um %lld", static_cast<long long>(state.encoder_translation_um)) &&
            debug_handler_helpers::append_format(response, sizeof(response), offset, " encoder_rotation_urad %lld", static_cast<long long>(state.encoder_rotation_urad)) &&
            debug_handler_helpers::append_format(response, sizeof(response), offset, " encoder_translation_sum_um %lld", static_cast<long long>(state.encoder_translation_sum_um)) &&
            debug_handler_helpers::append_format(response, sizeof(response), offset, " encoder_rotation_sum_urad %lld", static_cast<long long>(state.encoder_rotation_sum_urad)) &&
            debug_handler_helpers::append_format(response, sizeof(response), offset, " imu_translation_um %lld", static_cast<long long>(state.imu_translation_um)) &&
            debug_handler_helpers::append_format(response, sizeof(response), offset, " imu_rotation_urad %lld", static_cast<long long>(state.imu_rotation_urad)) &&
            debug_handler_helpers::append_format(response, sizeof(response), offset, " imu_translation_sum_um %lld", static_cast<long long>(state.imu_translation_sum_um)) &&
            debug_handler_helpers::append_format(response, sizeof(response), offset, " imu_rotation_sum_urad %lld", static_cast<long long>(state.imu_rotation_sum_urad)) &&
            debug_handler_helpers::append_format(response, sizeof(response), offset, " fused_translation_um %lld", static_cast<long long>(state.fused_translation_um)) &&
            debug_handler_helpers::append_format(response, sizeof(response), offset, " fused_rotation_urad %lld", static_cast<long long>(state.fused_rotation_urad)) &&
            debug_handler_helpers::append_format(response, sizeof(response), offset, " fused_translation_sum_um %lld", static_cast<long long>(state.fused_translation_sum_um)) &&
            debug_handler_helpers::append_format(response, sizeof(response), offset, " fused_rotation_sum_urad %lld", static_cast<long long>(state.fused_rotation_sum_urad)) &&
            debug_handler_helpers::append_format(response, sizeof(response), offset, " local_x_um %lld", static_cast<long long>(state.local_x_um)) &&
            debug_handler_helpers::append_format(response, sizeof(response), offset, " local_y_um %lld", static_cast<long long>(state.local_y_um)) &&
            debug_handler_helpers::append_format(response, sizeof(response), offset, " local_heading_urad %ld", static_cast<long>(state.local_heading_urad)) &&
            debug_handler_helpers::append_format(response, sizeof(response), offset, " local_update_id %lu", static_cast<unsigned long>(state.local_update_id)) &&
            debug_handler_helpers::append_format(response, sizeof(response), offset, " time_ms %lu", static_cast<unsigned long>(state.time_ms));

        if (formatted == false)
        {
            return false;
        }

        return handler_helpers::write_response_text(response);
    }
}
