#include "board_esp32_wroom32.hpp"

#include <Arduino.h>

#include "board_config.hpp"
#include "../core/control/robot_control.hpp"
#include "../platform/esp_uart_api.hpp"

namespace
{
    bool g_initialized = false;
}

namespace board_esp32_wroom32
{
    void init()
    {
        if (g_initialized == true)
        {
            return;
        }

        Serial.begin(board_config::usb_debug_baud_rate);

        pinMode(board_config::button_1_pin, INPUT);
        pinMode(board_config::button_2_pin, INPUT);

        esp_uart_api::init();

        robot_control::init();

        g_initialized = true;
    }

    void tick(std::uint32_t now_ms)
    {
        if (g_initialized == false)
        {
            return;
        }

        robot_control::tick(now_ms);
    }
}