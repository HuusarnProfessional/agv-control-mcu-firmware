#include "esp_uart_api.hpp"

#include <Arduino.h>

#include "../board/board_config.hpp"

namespace
{
    bool g_is_initialized = false;

    HardwareSerial *get_serial(esp_uart_api::uart_port port)
    {
        if (port == esp_uart_api::uart_port::motion_mcu)
        {
            return &Serial2;
        }

        if (port == esp_uart_api::uart_port::dwm1001)
        {
            return &Serial1;
        }

        return nullptr;
    }
}

namespace esp_uart_api
{
    void init()
    {
        Serial2.begin(
            board_config::motion_mcu_uart_baud_rate,
            SERIAL_8N1,
            board_config::motion_mcu_uart_rx_pin,
            board_config::motion_mcu_uart_tx_pin);

        Serial1.begin(
            board_config::dwm1001_uart_baud_rate,
            SERIAL_8N1,
            board_config::dwm1001_uart_rx_pin,
            board_config::dwm1001_uart_tx_pin);

        g_is_initialized = true;
    }

    uart_status write_bytes(uart_port port, const std::uint8_t *data, std::size_t length)
    {
        if (g_is_initialized == false)
        {
            return uart_status::not_initialized;
        }

        if (data == nullptr)
        {
            return uart_status::invalid_arg;
        }

        if (length == 0U)
        {
            return uart_status::ok;
        }

        HardwareSerial *serial = get_serial(port);

        if (serial == nullptr)
        {
            return uart_status::invalid_arg;
        }

        std::size_t written = serial->write(data, length);

        if (written != length)
        {
            return uart_status::write_failed;
        }

        return uart_status::ok;
    }

    std::size_t read_bytes(uart_port port, std::uint8_t *data_out, std::size_t capacity)
    {
        if (g_is_initialized == false)
        {
            return 0U;
        }

        if (data_out == nullptr)
        {
            return 0U;
        }

        if (capacity == 0U)
        {
            return 0U;
        }

        HardwareSerial *serial = get_serial(port);

        if (serial == nullptr)
        {
            return 0U;
        }

        std::size_t count = 0U;

        while (count < capacity)
        {
            if (serial->available() <= 0)
            {
                break;
            }

            int value = serial->read();

            if (value < 0)
            {
                break;
            }

            data_out[count] = static_cast<std::uint8_t>(value);
            count++;
        }

        return count;
    }
}
