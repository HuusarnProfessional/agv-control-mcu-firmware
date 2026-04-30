#include "motion_mcu_transport.hpp"

#include "../platform/esp_uart_api.hpp"
#include "motion_mcu_routes.hpp"

namespace motion_mcu_transport
{
    std::size_t read_bytes(std::uint8_t *data_out, std::size_t capacity)
    {
        return esp_uart_api::read_bytes(esp_uart_api::uart_port::motion_mcu, data_out, capacity);
    }

    transport_status write_packet(std::uint8_t payload_id, const std::uint8_t *payload_data, std::uint8_t payload_length)
    {
        if (payload_length > motion_mcu_routes::max_payload_length)
        {
            return transport_status::payload_too_large;
        }

        if ((payload_data == nullptr) && (payload_length > 0U))
        {
            return transport_status::invalid_arg;
        }

        std::uint8_t packet[3U + motion_mcu_routes::max_payload_length] = {};
        packet[0U] = motion_mcu_routes::packet_sync;
        packet[1U] = payload_id;
        packet[2U] = payload_length;

        for (std::uint8_t index = 0U; index < payload_length; index++)
        {
            packet[3U + index] = payload_data[index];
        }

        esp_uart_api::uart_status status = esp_uart_api::write_bytes(
            esp_uart_api::uart_port::motion_mcu,
            packet,
            static_cast<std::size_t>(3U + payload_length));

        if (status != esp_uart_api::uart_status::ok)
        {
            return transport_status::uart_error;
        }

        return transport_status::ok;
    }
}
