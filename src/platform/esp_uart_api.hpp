#pragma once

#include <cstddef>
#include <cstdint>

namespace esp_uart_api
{
    enum class uart_port : std::uint8_t
    {
        motion_mcu = 0U,
        dwm1001 = 1U
    };

    enum class uart_status : std::uint8_t
    {
        ok = 0U,
        invalid_arg,
        not_initialized,
        write_failed
    };

    void init();

    uart_status write_bytes(uart_port port, const std::uint8_t *data, std::size_t length);

    std::size_t read_bytes(uart_port port, std::uint8_t *data_out, std::size_t capacity);
}
