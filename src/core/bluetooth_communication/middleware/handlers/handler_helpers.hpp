#pragma once

#include <cerrno>
#include <climits>
#include <cstddef>
#include <cstdint>
#include <cstdlib>

#include "../../bluetooth_transport.hpp"

namespace handler_helpers
{
    template <std::size_t N>
    bool write_response(const char (&response)[N])
    {
        return bluetooth_transport::write_bytes(
                   reinterpret_cast<const std::uint8_t *>(response),
                   N - 1u) == bluetooth_transport::transport_status::ok;
    }

    inline bool parse_int16(const char *text, std::int16_t &value_out)
    {
        if (text == nullptr)
        {
            return false;
        }

        errno = 0;
        char *end_ptr = nullptr;
        const long parsed_value = std::strtol(text, &end_ptr, 10);

        if ((errno != 0) || (end_ptr == text) || (*end_ptr != '\0') || (parsed_value < SHRT_MIN) || (parsed_value > SHRT_MAX))
        {
            return false;
        }

        value_out = static_cast<std::int16_t>(parsed_value);
        return true;
    }

    inline bool parse_uint16(const char *text, std::uint16_t &value_out)
    {
        if (text == nullptr)
        {
            return false;
        }

        errno = 0;
        char *end_ptr = nullptr;
        const unsigned long parsed_value = std::strtoul(text, &end_ptr, 10);

        if ((errno != 0) || (end_ptr == text) || (*end_ptr != '\0') || (parsed_value > USHRT_MAX))
        {
            return false;
        }

        value_out = static_cast<std::uint16_t>(parsed_value);
        return true;
    }

    inline bool parse_int8(const char *text, std::int8_t &value_out)
    {
        if (text == nullptr)
        {
            return false;
        }

        errno = 0;
        char *end_ptr = nullptr;
        const long parsed_value = std::strtol(text, &end_ptr, 10);

        if ((errno != 0) || (end_ptr == text) || (*end_ptr != '\0') || (parsed_value < SCHAR_MIN) || (parsed_value > SCHAR_MAX))
        {
            return false;
        }

        value_out = static_cast<std::int8_t>(parsed_value);
        return true;
    }

    inline bool parse_uint8(const char *text, std::uint8_t &value_out)
    {
        if (text == nullptr)
        {
            return false;
        }

        errno = 0;
        char *end_ptr = nullptr;
        const unsigned long parsed_value = std::strtoul(text, &end_ptr, 10);

        if ((errno != 0) || (end_ptr == text) || (*end_ptr != '\0') || (parsed_value > UCHAR_MAX))
        {
            return false;
        }

        value_out = static_cast<std::uint8_t>(parsed_value);
        return true;
    }

    inline bool parse_bool(const char *text, bool &value_out)
    {
        if (text == nullptr)
        {
            return false;
        }

        if ((text[0] == 't') && (text[1] == 'r') && (text[2] == 'u') && (text[3] == 'e') && (text[4] == '\0'))
        {
            value_out = true;
            return true;
        }

        if ((text[0] == 'f') && (text[1] == 'a') && (text[2] == 'l') && (text[3] == 's') && (text[4] == 'e') && (text[5] == '\0'))
        {
            value_out = false;
            return true;
        }

        return false;
    }
}
