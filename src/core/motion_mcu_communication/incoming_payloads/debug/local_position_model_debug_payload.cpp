#include "local_position_model_debug_payload.hpp"

#include <Arduino.h>

#include "../../payload_helper_functions.hpp"
#include "../../state/debug/debug_state.hpp"

namespace local_position_model_debug_payload
{
    void handle(const std::uint8_t *payload_data, std::uint8_t payload_length)
    {
        motion_mcu_debug_state::local_position_model_debug_state state = motion_mcu_debug_state::get_local_position_model_debug();
        state.valid = false;
        state.received_time_ms = millis();

        const bool has_encoder_motion = payload_helper_functions::read_bool(payload_data, payload_length, 0U, state.has_encoder_motion);
        const bool has_imu_motion = payload_helper_functions::read_bool(payload_data, payload_length, 1U, state.has_imu_motion);
        const bool has_fused_translation = payload_helper_functions::read_bool(payload_data, payload_length, 2U, state.has_fused_translation);
        const bool has_fused_rotation = payload_helper_functions::read_bool(payload_data, payload_length, 3U, state.has_fused_rotation);
        const bool has_encoder_translation = payload_helper_functions::read_i64_le(payload_data, payload_length, 4U, state.encoder_translation_um);
        const bool has_encoder_rotation = payload_helper_functions::read_i64_le(payload_data, payload_length, 12U, state.encoder_rotation_urad);
        const bool has_encoder_translation_sum = payload_helper_functions::read_i64_le(payload_data, payload_length, 20U, state.encoder_translation_sum_um);
        const bool has_encoder_rotation_sum = payload_helper_functions::read_i64_le(payload_data, payload_length, 28U, state.encoder_rotation_sum_urad);
        const bool has_imu_translation = payload_helper_functions::read_i64_le(payload_data, payload_length, 36U, state.imu_translation_um);
        const bool has_imu_rotation = payload_helper_functions::read_i64_le(payload_data, payload_length, 44U, state.imu_rotation_urad);
        const bool has_imu_translation_sum = payload_helper_functions::read_i64_le(payload_data, payload_length, 52U, state.imu_translation_sum_um);
        const bool has_imu_rotation_sum = payload_helper_functions::read_i64_le(payload_data, payload_length, 60U, state.imu_rotation_sum_urad);
        const bool has_fused_translation_value = payload_helper_functions::read_i64_le(payload_data, payload_length, 68U, state.fused_translation_um);
        const bool has_fused_rotation_value = payload_helper_functions::read_i64_le(payload_data, payload_length, 76U, state.fused_rotation_urad);
        const bool has_fused_translation_sum = payload_helper_functions::read_i64_le(payload_data, payload_length, 84U, state.fused_translation_sum_um);
        const bool has_fused_rotation_sum = payload_helper_functions::read_i64_le(payload_data, payload_length, 92U, state.fused_rotation_sum_urad);
        const bool has_local_x = payload_helper_functions::read_i64_le(payload_data, payload_length, 100U, state.local_x_um);
        const bool has_local_y = payload_helper_functions::read_i64_le(payload_data, payload_length, 108U, state.local_y_um);
        const bool has_local_heading = payload_helper_functions::read_i32_le(payload_data, payload_length, 116U, state.local_heading_urad);
        const bool has_local_update_id = payload_helper_functions::read_u32_le(payload_data, payload_length, 120U, state.local_update_id);
        const bool has_time_ms = payload_helper_functions::read_u32_le(payload_data, payload_length, 124U, state.time_ms);

        if ((has_encoder_motion == false) ||
            (has_imu_motion == false) ||
            (has_fused_translation == false) ||
            (has_fused_rotation == false) ||
            (has_encoder_translation == false) ||
            (has_encoder_rotation == false) ||
            (has_encoder_translation_sum == false) ||
            (has_encoder_rotation_sum == false) ||
            (has_imu_translation == false) ||
            (has_imu_rotation == false) ||
            (has_imu_translation_sum == false) ||
            (has_imu_rotation_sum == false) ||
            (has_fused_translation_value == false) ||
            (has_fused_rotation_value == false) ||
            (has_fused_translation_sum == false) ||
            (has_fused_rotation_sum == false) ||
            (has_local_x == false) ||
            (has_local_y == false) ||
            (has_local_heading == false) ||
            (has_local_update_id == false) ||
            (has_time_ms == false))
        {
            return;
        }

        state.valid = true;
        motion_mcu_debug_state::set_local_position_model_debug(state);
    }
}
