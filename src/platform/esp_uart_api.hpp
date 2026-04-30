#pragma once

#include <cstddef>
#include <cstdint>

namespace esp_uart_api
{
    enum class uart_status : std::uint8_t
    {
        ok = 0,
        invalid_arg,
        not_initialized,
        write_failed
    };

    void init();

    uart_status write_bytes(std::uint8_t uart_id, const std::uint8_t *data, std::size_t length);

    std::size_t read_bytes(std::uint8_t uart_id, std::uint8_t *data_out, std::size_t capacity);

    std::size_t available_bytes(std::uint8_t uart_id);
}