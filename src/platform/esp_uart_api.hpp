#pragma once

#include <cstddef>
#include <cstdint>

#include "../board/board_esp32_wroom32.hpp"

namespace esp_uart_api
{
constexpr std::uint8_t motion_mcu_uart_id = board_esp32_wroom32::motion_mcu_uart_id;
constexpr std::uint8_t dwm1001_uart_id = board_esp32_wroom32::dwm1001_uart_id;

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
