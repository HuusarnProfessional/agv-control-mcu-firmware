#include "board_esp32_wroom32.hpp"

#include <Arduino.h>

#include "../core/bluetooth_communication/bluetooth_transport.hpp"
#include "../core/control/robot_control.hpp"
#include "../platform/esp_uart_api.hpp"

namespace
{

constexpr std::uint32_t usb_debug_baud_rate = 115200u;

constexpr std::uint8_t button_1_pin = 34u;
constexpr std::uint8_t button_2_pin = 35u;

constexpr const char *bluetooth_device_name = "first_mission_AGV_ESP32";

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

    Serial.begin(usb_debug_baud_rate);

    pinMode(button_1_pin, INPUT);
    pinMode(button_2_pin, INPUT);

    bluetooth_transport::init(bluetooth_device_name);
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
