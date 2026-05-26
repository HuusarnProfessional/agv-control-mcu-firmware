#include "../debug_handler_declarations.hpp"

#include <cstddef>

#include "../../../middleware_parse_helpers.hpp"
#include "../../handler_helpers.hpp"
#include "../../../../../motion_mcu_communication/motion_mcu_routes.hpp"
#include "../../../../../motion_mcu_communication/state/debug/debug_state.hpp"
#include "debug_handler_helpers.hpp"

namespace debug_handlers
{
    bool handle_get_imu_debug()
    {
        if (middleware_parse_helpers::read_end(debug_handler_helpers::timeout_us) == false)
        {
            return debug_handler_helpers::write_bad_format();
        }

        const std::uint8_t payload_id = static_cast<std::uint8_t>(motion_mcu_routes::incoming_payload_id::imu_debug);

        if (motion_mcu_debug_state::is_stream_enabled(payload_id) == false)
        {
            return debug_handler_helpers::write_stream_not_active("imu_debug");
        }

        const motion_mcu_debug_state::imu_debug_state state = motion_mcu_debug_state::get_imu_debug();

        if (state.valid == false)
        {
            return debug_handler_helpers::write_missing_data("imu_debug");
        }

        char response[512] = {};
        std::size_t offset = 0U;

        const bool formatted =
            debug_handler_helpers::append_format(response, sizeof(response), offset, "imu_debug") &&
            debug_handler_helpers::append_format(response, sizeof(response), offset, " gx_mdps %ld", static_cast<long>(state.gyro_mdps[0])) &&
            debug_handler_helpers::append_format(response, sizeof(response), offset, " gy_mdps %ld", static_cast<long>(state.gyro_mdps[1])) &&
            debug_handler_helpers::append_format(response, sizeof(response), offset, " gz_mdps %ld", static_cast<long>(state.gyro_mdps[2])) &&
            debug_handler_helpers::append_format(response, sizeof(response), offset, " ax_mg %ld", static_cast<long>(state.accel_mg[0])) &&
            debug_handler_helpers::append_format(response, sizeof(response), offset, " ay_mg %ld", static_cast<long>(state.accel_mg[1])) &&
            debug_handler_helpers::append_format(response, sizeof(response), offset, " az_mg %ld", static_cast<long>(state.accel_mg[2])) &&
            debug_handler_helpers::append_format(response, sizeof(response), offset, " mx_mgauss %ld", static_cast<long>(state.mag_mgauss[0])) &&
            debug_handler_helpers::append_format(response, sizeof(response), offset, " my_mgauss %ld", static_cast<long>(state.mag_mgauss[1])) &&
            debug_handler_helpers::append_format(response, sizeof(response), offset, " mz_mgauss %ld", static_cast<long>(state.mag_mgauss[2])) &&
            debug_handler_helpers::append_format(response, sizeof(response), offset, " raw_gx %d", static_cast<int>(state.raw_gyro[0])) &&
            debug_handler_helpers::append_format(response, sizeof(response), offset, " raw_gy %d", static_cast<int>(state.raw_gyro[1])) &&
            debug_handler_helpers::append_format(response, sizeof(response), offset, " raw_gz %d", static_cast<int>(state.raw_gyro[2])) &&
            debug_handler_helpers::append_format(response, sizeof(response), offset, " raw_ax %d", static_cast<int>(state.raw_accel[0])) &&
            debug_handler_helpers::append_format(response, sizeof(response), offset, " raw_ay %d", static_cast<int>(state.raw_accel[1])) &&
            debug_handler_helpers::append_format(response, sizeof(response), offset, " raw_az %d", static_cast<int>(state.raw_accel[2])) &&
            debug_handler_helpers::append_format(response, sizeof(response), offset, " raw_mx %d", static_cast<int>(state.raw_mag[0])) &&
            debug_handler_helpers::append_format(response, sizeof(response), offset, " raw_my %d", static_cast<int>(state.raw_mag[1])) &&
            debug_handler_helpers::append_format(response, sizeof(response), offset, " raw_mz %d", static_cast<int>(state.raw_mag[2])) &&
            debug_handler_helpers::append_format(response, sizeof(response), offset, " cgx_mdps %ld", static_cast<long>(state.calibrated_gyro_mdps[0])) &&
            debug_handler_helpers::append_format(response, sizeof(response), offset, " cgy_mdps %ld", static_cast<long>(state.calibrated_gyro_mdps[1])) &&
            debug_handler_helpers::append_format(response, sizeof(response), offset, " cgz_mdps %ld", static_cast<long>(state.calibrated_gyro_mdps[2])) &&
            debug_handler_helpers::append_format(response, sizeof(response), offset, " cax_mg %ld", static_cast<long>(state.calibrated_accel_mg[0])) &&
            debug_handler_helpers::append_format(response, sizeof(response), offset, " cay_mg %ld", static_cast<long>(state.calibrated_accel_mg[1])) &&
            debug_handler_helpers::append_format(response, sizeof(response), offset, " caz_mg %ld", static_cast<long>(state.calibrated_accel_mg[2])) &&
            debug_handler_helpers::append_format(response, sizeof(response), offset, " has_calibration %u", state.has_calibration ? 1U : 0U) &&
            debug_handler_helpers::append_format(response, sizeof(response), offset, " gyro_status %u", static_cast<unsigned>(state.gyro_status)) &&
            debug_handler_helpers::append_format(response, sizeof(response), offset, " accel_status %u", static_cast<unsigned>(state.accel_status)) &&
            debug_handler_helpers::append_format(response, sizeof(response), offset, " mag_status %u", static_cast<unsigned>(state.mag_status)) &&
            debug_handler_helpers::append_format(response, sizeof(response), offset, " time_ms %lu", static_cast<unsigned long>(state.time_ms));

        if (formatted == false)
        {
            return false;
        }

        return handler_helpers::write_response_text(response);
    }
}
