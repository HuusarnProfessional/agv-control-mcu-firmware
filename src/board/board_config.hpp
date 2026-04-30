#pragma once

#include <cstdint>

namespace board_config
{
    constexpr std::uint32_t usb_debug_baud_rate = 115200U;

    constexpr std::uint8_t button_1_pin = 34U;
    constexpr std::uint8_t button_2_pin = 35U;

    constexpr std::uint8_t motion_mcu_uart_rx_pin = 16U;
    constexpr std::uint8_t motion_mcu_uart_tx_pin = 17U;
    constexpr std::uint32_t motion_mcu_uart_baud_rate = 115200U;

    constexpr std::uint8_t dwm1001_uart_rx_pin = 26U;
    constexpr std::uint8_t dwm1001_uart_tx_pin = 25U;
    constexpr std::uint32_t dwm1001_uart_baud_rate = 115200U;

    constexpr std::uint8_t oled_sda_pin = 21U;
    constexpr std::uint8_t oled_scl_pin = 22U;
}
