#include "middleware_parse_helpers.hpp"

#include <Arduino.h>

#include <cerrno>
#include <climits>
#include <cstdlib>

#include "middleware_handler_input_bridge.hpp"

namespace
{
    bool read_next_byte(std::uint8_t &byte_out, std::uint32_t timeout_us)
    {
        const std::uint32_t start_time_us = micros();

        while ((micros() - start_time_us) < timeout_us)
        {
            if (middleware_handler_input_bridge::read_byte(byte_out) == true)
            {
                return true;
            }
        }

        return false;
    }

    bool parse_signed_long(const char *text, long &value_out)
    {
        if (text == nullptr)
        {
            return false;
        }

        errno = 0;
        char *end_ptr = nullptr;
        const long parsed_value = std::strtol(text, &end_ptr, 10);

        if ((errno != 0) || (end_ptr == text) || (*end_ptr != '\0'))
        {
            return false;
        }

        value_out = parsed_value;
        return true;
    }

    bool parse_unsigned_long(const char *text, unsigned long &value_out)
    {
        if (text == nullptr)
        {
            return false;
        }

        errno = 0;
        char *end_ptr = nullptr;
        const unsigned long parsed_value = std::strtoul(text, &end_ptr, 10);

        if ((errno != 0) || (end_ptr == text) || (*end_ptr != '\0'))
        {
            return false;
        }

        value_out = parsed_value;
        return true;
    }
}

namespace middleware_parse_helpers
{
    bool discard_until_end(std::uint32_t timeout_us)
    {
        std::uint8_t byte_value = 0;

        while (read_next_byte(byte_value, timeout_us) == true)
        {
            if (byte_value == static_cast<std::uint8_t>(')'))
            {
                return true;
            }
        }

        return false;
    }

    bool read_end(std::uint32_t timeout_us)
    {
        std::uint8_t byte_value = 0;

        if (read_next_byte(byte_value, timeout_us) == false)
        {
            return false;
        }

        return byte_value == static_cast<std::uint8_t>(')');
    }

    bool read_int16_and_end(std::int16_t &value_out, std::uint32_t timeout_us)
    {
        char token_buffer[16] = {};
        bool ended = false;

        if (read_until_comma_or_end(token_buffer, sizeof(token_buffer), ended, timeout_us) == false)
        {
            return false;
        }

        if (ended == false)
        {
            return false;
        }

        long parsed_value = 0;

        if (parse_signed_long(token_buffer, parsed_value) == false)
        {
            return false;
        }

        if ((parsed_value < SHRT_MIN) || (parsed_value > SHRT_MAX))
        {
            return false;
        }

        value_out = static_cast<std::int16_t>(parsed_value);
        return true;
    }

    bool read_uint16_and_end(std::uint16_t &value_out, std::uint32_t timeout_us)
    {
        char token_buffer[16] = {};
        bool ended = false;

        if (read_until_comma_or_end(token_buffer, sizeof(token_buffer), ended, timeout_us) == false)
        {
            return false;
        }

        if (ended == false)
        {
            return false;
        }

        unsigned long parsed_value = 0;

        if (parse_unsigned_long(token_buffer, parsed_value) == false)
        {
            return false;
        }

        if (parsed_value > USHRT_MAX)
        {
            return false;
        }

        value_out = static_cast<std::uint16_t>(parsed_value);
        return true;
    }

    bool read_int8_and_end(std::int8_t &value_out, std::uint32_t timeout_us)
    {
        char token_buffer[8] = {};
        bool ended = false;

        if (read_until_comma_or_end(token_buffer, sizeof(token_buffer), ended, timeout_us) == false)
        {
            return false;
        }

        if (ended == false)
        {
            return false;
        }

        long parsed_value = 0;

        if (parse_signed_long(token_buffer, parsed_value) == false)
        {
            return false;
        }

        if ((parsed_value < SCHAR_MIN) || (parsed_value > SCHAR_MAX))
        {
            return false;
        }

        value_out = static_cast<std::int8_t>(parsed_value);
        return true;
    }

    bool read_uint8_and_end(std::uint8_t &value_out, std::uint32_t timeout_us)
    {
        char token_buffer[8] = {};
        bool ended = false;

        if (read_until_comma_or_end(token_buffer, sizeof(token_buffer), ended, timeout_us) == false)
        {
            return false;
        }

        if (ended == false)
        {
            return false;
        }

        unsigned long parsed_value = 0;

        if (parse_unsigned_long(token_buffer, parsed_value) == false)
        {
            return false;
        }

        if (parsed_value > UCHAR_MAX)
        {
            return false;
        }

        value_out = static_cast<std::uint8_t>(parsed_value);
        return true;
    }

    bool read_bool_and_end(bool &value_out, std::uint32_t timeout_us)
    {
        char token_buffer[8] = {};
        bool ended = false;

        if (read_until_comma_or_end(token_buffer, sizeof(token_buffer), ended, timeout_us) == false)
        {
            return false;
        }

        if (ended == false)
        {
            return false;
        }

        if ((token_buffer[0] == 't') && (token_buffer[1] == 'r') && (token_buffer[2] == 'u') &&
            (token_buffer[3] == 'e') && (token_buffer[4] == '\0'))
        {
            value_out = true;
            return true;
        }

        if ((token_buffer[0] == 'f') && (token_buffer[1] == 'a') && (token_buffer[2] == 'l') &&
            (token_buffer[3] == 's') && (token_buffer[4] == 'e') && (token_buffer[5] == '\0'))
        {
            value_out = false;
            return true;
        }

        return false;
    }

    bool read_csv_text_and_end(char *buffer_out, std::size_t capacity, std::uint32_t timeout_us)
    {
        if ((buffer_out == nullptr) || (capacity == 0))
        {
            return false;
        }

        std::size_t length = 0;
        std::uint8_t byte_value = 0;

        while (read_next_byte(byte_value, timeout_us) == true)
        {
            if (byte_value == static_cast<std::uint8_t>(')'))
            {
                buffer_out[length] = '\0';
                return true;
            }

            if (length + 1 >= capacity)
            {
                return false;
            }

            buffer_out[length] = static_cast<char>(byte_value);
            ++length;
        }

        return false;
    }

    bool read_until_comma_or_end(char *buffer_out, std::size_t capacity, bool &ended_out, std::uint32_t timeout_us)
    {
        if ((buffer_out == nullptr) || (capacity == 0))
        {
            return false;
        }

        ended_out = false;

        std::size_t length = 0;
        std::uint8_t byte_value = 0;

        while (read_next_byte(byte_value, timeout_us) == true)
        {
            if (byte_value == static_cast<std::uint8_t>(')'))
            {
                buffer_out[length] = '\0';
                ended_out = true;
                return true;
            }

            if (byte_value == static_cast<std::uint8_t>(','))
            {
                buffer_out[length] = '\0';
                return true;
            }

            if (length + 1 >= capacity)
            {
                return false;
            }

            buffer_out[length] = static_cast<char>(byte_value);
            ++length;
        }

        return false;
    }

    bool read_command_until_comma_or_end(char *buffer_out, std::size_t capacity, bool &ended_out, std::uint32_t timeout_us)
    {
        if ((buffer_out == nullptr) || (capacity == 0))
        {
            return false;
        }

        ended_out = false;

        std::size_t length = 0;
        std::uint8_t byte_value = 0;
        std::int32_t nested_parenthesis_depth = 0;

        while (read_next_byte(byte_value, timeout_us) == true)
        {
            if (byte_value == static_cast<std::uint8_t>('('))
            {
                ++nested_parenthesis_depth;
            }
            else if (byte_value == static_cast<std::uint8_t>(')'))
            {
                if (nested_parenthesis_depth == 0)
                {
                    buffer_out[length] = '\0';
                    ended_out = true;
                    return true;
                }

                --nested_parenthesis_depth;
            }
            else if ((byte_value == static_cast<std::uint8_t>(',')) && (nested_parenthesis_depth == 0))
            {
                buffer_out[length] = '\0';
                return true;
            }

            if (length + 1 >= capacity)
            {
                return false;
            }

            buffer_out[length] = static_cast<char>(byte_value);
            ++length;
        }

        return false;
    }

    bool read_binary(std::uint8_t *buffer_out, std::size_t length, std::uint32_t timeout_us)
    {
        if ((buffer_out == nullptr) && (length > 0))
        {
            return false;
        }

        for (std::size_t index = 0; index < length; ++index)
        {
            if (read_next_byte(buffer_out[index], timeout_us) == false)
            {
                return false;
            }
        }

        return true;
    }
}
