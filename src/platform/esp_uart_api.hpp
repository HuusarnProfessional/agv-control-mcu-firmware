#pragma once

#include <cstddef>
#include <cstdint>

namespace esp_uart_api
{

enum class uart_channel : std::uint8_t
{
    motion_mcu = 0u,
    dwm1001 = 1u
};

enum class uart_status : std::uint8_t
{
    ok = 0u,
    invalid_arg,
    not_initialized,
    write_failed
};

void init();

uart_status write_bytes(uart_channel channel, const std::uint8_t *data, std::size_t length);

std::size_t read_bytes(uart_channel channel, std::uint8_t *data_out, std::size_t capacity);

std::size_t available_bytes(uart_channel channel);

}
