#pragma once

#include <cstddef>
#include <cstdint>

namespace motion_mcu_transport
{
    enum class transport_status : std::uint8_t
    {
        ok = 0U,
        invalid_arg,
        payload_too_large,
        uart_error
    };

    std::size_t read_bytes(std::uint8_t *data_out, std::size_t capacity);

    transport_status write_packet(std::uint8_t payload_id, const std::uint8_t *payload_data, std::uint8_t payload_length);
}
