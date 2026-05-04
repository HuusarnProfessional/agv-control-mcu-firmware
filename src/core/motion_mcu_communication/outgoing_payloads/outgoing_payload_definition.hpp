#pragma once

#include <cstdint>

#include "../../../platform/esp_uart_api.hpp"
#include "../motion_mcu_routes.hpp"

namespace outgoing_payload_definition
{
    struct payload_buffer
    {
        std::uint8_t payload_id;
        std::uint8_t payload_length;
        std::uint8_t payload_data[64U];
    };

    inline bool send_payload(const payload_buffer &payload)
    {
        if (payload.payload_length > motion_mcu_routes::max_payload_length)
        {
            return false;
        }

        std::uint8_t packet[67U] = {};
        packet[0] = motion_mcu_routes::packet_sync;
        packet[1] = payload.payload_id;
        packet[2] = payload.payload_length;

        for (std::uint8_t index = 0U; index < payload.payload_length; index++)
        {
            packet[3U + index] = payload.payload_data[index];
        }

        const esp_uart_api::uart_status status = esp_uart_api::write_bytes(
            esp_uart_api::motion_mcu_uart_id,
            packet,
            static_cast<std::size_t>(3U + payload.payload_length));

        return status == esp_uart_api::uart_status::ok;
    }
}
