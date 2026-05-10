#pragma once

#include <cstdint>

namespace board_esp32_wroom32
{

constexpr std::uint8_t motion_mcu_uart_id = 2u;
constexpr std::uint8_t motion_mcu_uart_rx_pin = 16u;
constexpr std::uint8_t motion_mcu_uart_tx_pin = 17u;
constexpr std::uint32_t motion_mcu_uart_baud_rate = 115200u;

constexpr std::uint8_t dwm1001_uart_id = 1u;
constexpr std::uint8_t dwm1001_uart_rx_pin = 26u;
constexpr std::uint8_t dwm1001_uart_tx_pin = 25u;
constexpr std::uint32_t dwm1001_uart_baud_rate = 115200u;

void init();

void tick(std::uint32_t now_ms);

}
