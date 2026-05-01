#include "bluetooth_transport.hpp"

#include <Arduino.h>
#include <BluetoothSerial.h>

namespace
{
    BluetoothSerial g_bluetooth_serial;
    bool g_initialized = false;
}

namespace bluetooth_transport
{
    transport_status init(const char *device_name)
    {
        if (g_initialized == true)
        {
            return transport_status::ok;
        }

        if (device_name == nullptr)
        {
            return transport_status::invalid_arg;
        }

        bool started = g_bluetooth_serial.begin(device_name);

        if (started == false)
        {
            return transport_status::start_failed;
        }

        g_initialized = true;

        return transport_status::ok;
    }

    bool is_connected()
    {
        if (g_initialized == false)
        {
            return false;
        }

        return g_bluetooth_serial.hasClient();
    }

    std::size_t available_bytes()
    {
        if (g_initialized == false)
        {
            return 0;
        }

        int available_count = g_bluetooth_serial.available();

        if (available_count <= 0)
        {
            return 0;
        }

        return static_cast<std::size_t>(available_count);
    }

    bool read_byte(std::uint8_t &byte_out)
    {
        if (g_initialized == false)
        {
            return false;
        }

        if (g_bluetooth_serial.available() <= 0)
        {
            return false;
        }

        int byte_value = g_bluetooth_serial.read();

        if (byte_value < 0)
        {
            return false;
        }

        byte_out = static_cast<std::uint8_t>(byte_value);

        return true;
    }

    transport_status write_bytes(const std::uint8_t *data, std::size_t length)
    {
        if (g_initialized == false)
        {
            return transport_status::not_initialized;
        }

        if ((data == nullptr) && (length > 0))
        {
            return transport_status::invalid_arg;
        }

        if (length == 0)
        {
            return transport_status::ok;
        }

        std::size_t bytes_written = g_bluetooth_serial.write(data, length);

        if (bytes_written != length)
        {
            return transport_status::write_failed;
        }

        return transport_status::ok;
    }
}
