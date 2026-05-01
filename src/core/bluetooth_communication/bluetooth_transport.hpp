#pragma once

#include <cstddef>
#include <cstdint>

namespace bluetooth_transport
{
    enum class transport_status : std::uint8_t
    {
        ok = 0,
        invalid_arg,
        not_initialized,
        start_failed,
        write_failed
    };

    transport_status init(const char *device_name);

    bool is_connected();

    std::size_t available_bytes();

    bool read_byte(std::uint8_t &byte_out);

    transport_status write_bytes(const std::uint8_t *data, std::size_t length);
}
