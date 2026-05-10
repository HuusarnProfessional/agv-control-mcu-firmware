#include "esp_uart_api.hpp"

#include <Arduino.h>

#include "../board/board_esp32_wroom32.hpp"

namespace
{

HardwareSerial g_motion_mcu_serial(board_esp32_wroom32::motion_mcu_uart_id);
HardwareSerial g_dwm1001_serial(board_esp32_wroom32::dwm1001_uart_id);

struct uart_context
{
    esp_uart_api::uart_channel channel;
    HardwareSerial *serial;
    bool initialized;
};

uart_context g_motion_mcu_context =
{
    esp_uart_api::uart_channel::motion_mcu,
    &g_motion_mcu_serial,
    false
};

uart_context g_dwm1001_context =
{
    esp_uart_api::uart_channel::dwm1001,
    &g_dwm1001_serial,
    false
};

uart_context *find_context(esp_uart_api::uart_channel channel)
{
    if (g_motion_mcu_context.channel == channel)
    {
        return &g_motion_mcu_context;
    }

    if (g_dwm1001_context.channel == channel)
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
        board_esp32_wroom32::motion_mcu_uart_baud_rate,
        board_esp32_wroom32::motion_mcu_uart_rx_pin,
        board_esp32_wroom32::motion_mcu_uart_tx_pin
    );

    begin_context(
        g_dwm1001_context,
        board_esp32_wroom32::dwm1001_uart_baud_rate,
        board_esp32_wroom32::dwm1001_uart_rx_pin,
        board_esp32_wroom32::dwm1001_uart_tx_pin
    );
}

uart_status write_bytes(
    uart_channel channel,
    const std::uint8_t *data,
    std::size_t length
)
{
    uart_context *context = find_context(channel);

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

std::size_t read_bytes(
    uart_channel channel,
    std::uint8_t *data_out,
    std::size_t capacity
)
{
    uart_context *context = find_context(channel);

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

std::size_t available_bytes(uart_channel channel)
{
    uart_context *context = find_context(channel);

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
