#include "esp_uart_api.hpp"

#include <Arduino.h>

namespace
{
    constexpr std::uint32_t motion_mcu_uart_baud_rate = 115200u;
    constexpr std::uint8_t motion_mcu_uart_rx_pin = 16u;
    constexpr std::uint8_t motion_mcu_uart_tx_pin = 17u;
    constexpr std::uint32_t dwm1001_uart_baud_rate = 115200u;
    constexpr std::uint8_t dwm1001_uart_rx_pin = 26u;
    constexpr std::uint8_t dwm1001_uart_tx_pin = 25u;

    HardwareSerial g_motion_mcu_serial(esp_uart_api::motion_mcu_uart_id);
    HardwareSerial g_dwm1001_serial(esp_uart_api::dwm1001_uart_id);

    struct uart_context
    {
        std::uint8_t uart_id;
        HardwareSerial *serial;
        bool initialized;
    };

    uart_context g_motion_mcu_context =
    {
        esp_uart_api::motion_mcu_uart_id,
        &g_motion_mcu_serial,
        false
    };

    uart_context g_dwm1001_context =
    {
        esp_uart_api::dwm1001_uart_id,
        &g_dwm1001_serial,
        false
    };

    uart_context *find_context(std::uint8_t uart_id)
    {
        if (g_motion_mcu_context.uart_id == uart_id)
        {
            return &g_motion_mcu_context;
        }

        if (g_dwm1001_context.uart_id == uart_id)
        {
            return &g_dwm1001_context;
        }

        return nullptr;
    }

    void begin_context(uart_context &context, std::uint32_t baud_rate, std::uint8_t rx_pin, std::uint8_t tx_pin)
    {
        context.serial->begin(
            baud_rate,
            SERIAL_8N1,
            rx_pin,
            tx_pin
        );

        context.initialized = true;
    }
}

namespace esp_uart_api
{
    void init()
    {
        begin_context(
            g_motion_mcu_context,
            motion_mcu_uart_baud_rate,
            motion_mcu_uart_rx_pin,
            motion_mcu_uart_tx_pin
        );

        begin_context(
            g_dwm1001_context,
            dwm1001_uart_baud_rate,
            dwm1001_uart_rx_pin,
            dwm1001_uart_tx_pin
        );
    }

    uart_status write_bytes(std::uint8_t uart_id, const std::uint8_t *data, std::size_t length)
    {
        uart_context *context = find_context(uart_id);

        if (context == nullptr)
        {
            return uart_status::invalid_arg;
        }

        if (context->initialized == false)
        {
            return uart_status::not_initialized;
        }

        if ((data == nullptr) && (length > 0u))
        {
            return uart_status::invalid_arg;
        }

        if (length == 0u)
        {
            return uart_status::ok;
        }

        std::size_t bytes_written = context->serial->write(data, length);

        if (bytes_written != length)
        {
            return uart_status::write_failed;
        }

        return uart_status::ok;
    }

    std::size_t read_bytes(std::uint8_t uart_id, std::uint8_t *data_out, std::size_t capacity)
    {
        uart_context *context = find_context(uart_id);

        if (context == nullptr)
        {
            return 0u;
        }

        if (context->initialized == false)
        {
            return 0u;
        }

        if (data_out == nullptr)
        {
            return 0u;
        }

        if (capacity == 0u)
        {
            return 0u;
        }

        std::size_t bytes_read = 0u;

        while ((context->serial->available() > 0) && (bytes_read < capacity))
        {
            int byte_value = context->serial->read();

            if (byte_value < 0)
            {
                break;
            }

            data_out[bytes_read] = static_cast<std::uint8_t>(byte_value);
            bytes_read++;
        }

        return bytes_read;
    }

    std::size_t available_bytes(std::uint8_t uart_id)
    {
        uart_context *context = find_context(uart_id);

        if (context == nullptr)
        {
            return 0u;
        }

        if (context->initialized == false)
        {
            return 0u;
        }

        int available_count = context->serial->available();

        if (available_count < 0)
        {
            return 0u;
        }

        return static_cast<std::size_t>(available_count);
    }
}
