#include "imu_debug_payload.hpp"

#include <Arduino.h>

#include "../../payload_helper_functions.hpp"
#include "../../state/debug/debug_state.hpp"

namespace imu_debug_payload
{
    void handle(const std::uint8_t *payload_data, std::uint8_t payload_length)
    {
        motion_mcu_debug_state::imu_debug_state state = motion_mcu_debug_state::get_imu_debug();
        state.valid = false;
        state.received_time_ms = millis();

        std::size_t offset = 0U;

        for (std::uint8_t axis = 0U; axis < 3U; ++axis)
        {
            if (payload_helper_functions::read_i32_le(payload_data, payload_length, offset, state.gyro_mdps[axis]) == false)
            {
                return;
            }

            offset += 4U;
        }

        for (std::uint8_t axis = 0U; axis < 3U; ++axis)
        {
            if (payload_helper_functions::read_i32_le(payload_data, payload_length, offset, state.accel_mg[axis]) == false)
            {
                return;
            }

            offset += 4U;
        }

        for (std::uint8_t axis = 0U; axis < 3U; ++axis)
        {
            if (payload_helper_functions::read_i32_le(payload_data, payload_length, offset, state.mag_mgauss[axis]) == false)
            {
                return;
            }

            offset += 4U;
        }

        for (std::uint8_t axis = 0U; axis < 3U; ++axis)
        {
            if (payload_helper_functions::read_i16_le(payload_data, payload_length, offset, state.raw_gyro[axis]) == false)
            {
                return;
            }

            offset += 2U;
        }

        for (std::uint8_t axis = 0U; axis < 3U; ++axis)
        {
            if (payload_helper_functions::read_i16_le(payload_data, payload_length, offset, state.raw_accel[axis]) == false)
            {
                return;
            }

            offset += 2U;
        }

        for (std::uint8_t axis = 0U; axis < 3U; ++axis)
        {
            if (payload_helper_functions::read_i16_le(payload_data, payload_length, offset, state.raw_mag[axis]) == false)
            {
                return;
            }

            offset += 2U;
        }

        for (std::uint8_t axis = 0U; axis < 3U; ++axis)
        {
            if (payload_helper_functions::read_i32_le(payload_data, payload_length, offset, state.calibrated_gyro_mdps[axis]) == false)
            {
                return;
            }

            offset += 4U;
        }

        for (std::uint8_t axis = 0U; axis < 3U; ++axis)
        {
            if (payload_helper_functions::read_i32_le(payload_data, payload_length, offset, state.calibrated_accel_mg[axis]) == false)
            {
                return;
            }

            offset += 4U;
        }

        const bool has_calibration = payload_helper_functions::read_bool(payload_data, payload_length, offset, state.has_calibration);
        const bool has_gyro_status = payload_helper_functions::read_u8(payload_data, payload_length, offset + 1U, state.gyro_status);
        const bool has_accel_status = payload_helper_functions::read_u8(payload_data, payload_length, offset + 2U, state.accel_status);
        const bool has_mag_status = payload_helper_functions::read_u8(payload_data, payload_length, offset + 3U, state.mag_status);
        const bool has_time_ms = payload_helper_functions::read_u32_le(payload_data, payload_length, offset + 4U, state.time_ms);

        if ((has_calibration == false) || (has_gyro_status == false) || (has_accel_status == false) || (has_mag_status == false) || (has_time_ms == false))
        {
            return;
        }

        state.valid = true;
        motion_mcu_debug_state::set_imu_debug(state);
    }
}
