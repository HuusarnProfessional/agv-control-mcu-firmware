#include "board_esp32_wroom32.hpp"

#include <Arduino.h>

#include "board_config.hpp"
#include "../platform/esp_uart_api.hpp"
#include "../core/control/robot_control.hpp"

namespace board_esp32_wroom32
{
    void init()
    {
        Serial.begin(board_config::usb_debug_baud_rate);

        pinMode(board_config::button_1_pin, INPUT);
        pinMode(board_config::button_2_pin, INPUT);

        esp_uart_api::init();
        robot_control::init();
    }

    void tick(std::uint32_t now_ms)
    {
        robot_control::tick(now_ms);
    }
}
