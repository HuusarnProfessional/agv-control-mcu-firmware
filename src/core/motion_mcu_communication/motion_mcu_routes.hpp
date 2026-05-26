#pragma once

#include <cstdint>

namespace motion_mcu_routes
{
    constexpr std::uint8_t packet_sync = 0xA5U;
    constexpr std::uint8_t max_payload_length = 128U;

    enum class incoming_payload_id : std::uint8_t
    {
        local_position = 0x01U,
        safety_status = 0x02U,
        power_status = 0x03U,
        motion_primitive_status = 0x04U,
        encoder_debug = 0x11U,
        imu_debug = 0x12U,
        obstacle_debug = 0x13U,
        voltage_debug = 0x14U,
        motion_debug = 0x1DU,
        local_position_model_debug = 0x1EU
    };

    enum class outgoing_payload_id : std::uint8_t
    {
        motion_command = 0x21U,
        start_imu_calibration = 0x22U,
        clear_imu_calibration = 0x23U,
        debug_stream_control = 0x24U,
        trailer_status = 0x25U,
        unlock_safe_guard = 0x26U,
        lock_safe_guard = 0x27U,
        obstacle_safety_control = 0x28U,
        rotate_delta = 0x29U,
        drive_forward = 0x2AU,
        pause = 0x2BU,
        heartbeat = 0x2CU,
        position_correction = 0x30U,
        obstacle_margin_control = 0x31U
    };
}
