#pragma once

#include <cstdint>

namespace motion_mcu_debug_state
{
    constexpr std::uint8_t encoder_capacity = 4U;
    constexpr std::uint8_t obstacle_capacity = 2U;

    struct encoder_debug_state
    {
        bool valid = false;
        bool stream_enabled = false;
        std::uint32_t received_time_ms = 0U;
        std::uint8_t count = 0U;
        std::uint16_t raw[encoder_capacity] = {};
        std::uint32_t angle_mdeg[encoder_capacity] = {};
        std::uint32_t time_ms[encoder_capacity] = {};
        std::uint8_t status[encoder_capacity] = {};
    };

    struct imu_debug_state
    {
        bool valid = false;
        bool stream_enabled = false;
        std::uint32_t received_time_ms = 0U;
        std::int32_t gyro_mdps[3] = {};
        std::int32_t accel_mg[3] = {};
        std::int32_t mag_mgauss[3] = {};
        std::int16_t raw_gyro[3] = {};
        std::int16_t raw_accel[3] = {};
        std::int16_t raw_mag[3] = {};
        std::int32_t calibrated_gyro_mdps[3] = {};
        std::int32_t calibrated_accel_mg[3] = {};
        bool has_calibration = false;
        std::uint8_t gyro_status = 0U;
        std::uint8_t accel_status = 0U;
        std::uint8_t mag_status = 0U;
        std::uint32_t time_ms = 0U;
    };

    struct obstacle_debug_state
    {
        bool valid = false;
        bool stream_enabled = false;
        std::uint32_t received_time_ms = 0U;
        std::uint8_t count = 0U;
        std::uint32_t distance_mm[obstacle_capacity] = {};
        std::uint32_t time_ms[obstacle_capacity] = {};
        std::uint8_t status[obstacle_capacity] = {};
    };

    struct voltage_debug_state
    {
        bool valid = false;
        bool stream_enabled = false;
        std::uint32_t received_time_ms = 0U;
        std::uint16_t raw_adc = 0U;
        std::uint32_t voltage_mv = 0U;
        std::uint32_t time_ms = 0U;
        std::uint8_t status = 0U;
    };

    struct motion_debug_state
    {
        bool valid = false;
        bool stream_enabled = false;
        std::uint32_t received_time_ms = 0U;
        bool drive_enabled = false;
        bool motion_session_active = false;
        bool safe_guard_latched = false;
        bool motion_command_stale = false;
        bool has_pose = false;
        bool pose_is_fresh = false;
        bool heading_feedback_active = false;
        bool has_not_ready_feedback = false;
        bool has_invalid_feedback = false;
        std::int32_t commanded_linear_velocity_mm_s = 0;
        std::int32_t commanded_yaw_rate_mdeg_s = 0;
        std::int32_t corrected_yaw_rate_mdeg_s = 0;
        std::int32_t measured_yaw_rate_mdeg_s = 0;
        std::int32_t outer_correction_mdeg_s = 0;
        std::uint16_t pose_confidence_heading = 0U;
        std::uint32_t pose_age_ms = 0U;
        std::int32_t wheel_targets_mm_s[4] = {};
        std::int32_t wheel_speeds_mm_s[4] = {};
        std::uint32_t wheel_sample_ids[4] = {};
        std::uint32_t wheel_sample_age_ms[4] = {};
        bool wheel_has_new_sample[4] = {};
        bool wheel_has_measured_speed[4] = {};
        std::int16_t wheel_drive_u[4] = {};
        std::uint32_t time_ms = 0U;
    };

    struct local_position_model_debug_state
    {
        bool valid = false;
        bool stream_enabled = false;
        std::uint32_t received_time_ms = 0U;
        bool has_encoder_motion = false;
        bool has_imu_motion = false;
        bool has_fused_translation = false;
        bool has_fused_rotation = false;
        std::int64_t encoder_translation_um = 0;
        std::int64_t encoder_rotation_urad = 0;
        std::int64_t encoder_translation_sum_um = 0;
        std::int64_t encoder_rotation_sum_urad = 0;
        std::int64_t imu_translation_um = 0;
        std::int64_t imu_rotation_urad = 0;
        std::int64_t imu_translation_sum_um = 0;
        std::int64_t imu_rotation_sum_urad = 0;
        std::int64_t fused_translation_um = 0;
        std::int64_t fused_rotation_urad = 0;
        std::int64_t fused_translation_sum_um = 0;
        std::int64_t fused_rotation_sum_urad = 0;
        std::int64_t local_x_um = 0;
        std::int64_t local_y_um = 0;
        std::int32_t local_heading_urad = 0;
        std::uint32_t local_update_id = 0U;
        std::uint32_t time_ms = 0U;
    };

    void init();

    void set_stream_enabled(std::uint8_t payload_id, bool is_enabled);
    bool is_stream_enabled(std::uint8_t payload_id);

    void set_encoder_debug(const encoder_debug_state &state);
    void set_imu_debug(const imu_debug_state &state);
    void set_obstacle_debug(const obstacle_debug_state &state);
    void set_voltage_debug(const voltage_debug_state &state);
    void set_motion_debug(const motion_debug_state &state);
    void set_local_position_model_debug(const local_position_model_debug_state &state);

    encoder_debug_state get_encoder_debug();
    imu_debug_state get_imu_debug();
    obstacle_debug_state get_obstacle_debug();
    voltage_debug_state get_voltage_debug();
    motion_debug_state get_motion_debug();
    local_position_model_debug_state get_local_position_model_debug();
}
