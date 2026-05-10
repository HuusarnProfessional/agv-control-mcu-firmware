#pragma once

#include <cstdarg>
#include <cstddef>
#include <cstdint>
#include <cstdio>

#include "../../handler_helpers.hpp"

namespace debug_handler_helpers
{
    constexpr std::uint32_t timeout_us = 50000u;

    inline bool write_bad_format()
    {
        return handler_helpers::write_response_text("err bad_format");
    }

    inline bool write_stream_not_active(const char *stream_name)
    {
        if (stream_name == nullptr)
        {
            return false;
        }

        char response[64] = {};
        const int response_length = std::snprintf(response, sizeof(response), "err %s_not_active", stream_name);

        if ((response_length <= 0) || (static_cast<std::size_t>(response_length) >= sizeof(response)))
        {
            return false;
        }

        return handler_helpers::write_response_text(response);
    }

    inline bool write_missing_data(const char *name)
    {
        if (name == nullptr)
        {
            return false;
        }

        char response[64] = {};
        const int response_length = std::snprintf(response, sizeof(response), "err no_%s", name);

        if ((response_length <= 0) || (static_cast<std::size_t>(response_length) >= sizeof(response)))
        {
            return false;
        }

        return handler_helpers::write_response_text(response);
    }

    inline bool append_format(char *buffer, std::size_t capacity, std::size_t &offset, const char *format, ...)
    {
        if ((buffer == nullptr) || (format == nullptr) || (offset >= capacity))
        {
            return false;
        }

        va_list args;
        va_start(args, format);
        const int written = std::vsnprintf(buffer + offset, capacity - offset, format, args);
        va_end(args);

        if (written <= 0)
        {
            return false;
        }

        const std::size_t written_size = static_cast<std::size_t>(written);

        if ((offset + written_size) >= capacity)
        {
            return false;
        }

        offset += written_size;
        return true;
    }

    inline bool format_axis3_i32_line(const char *name, const std::int32_t values[3], char *buffer_out, std::size_t capacity)
    {
        if ((name == nullptr) || (values == nullptr) || (buffer_out == nullptr) || (capacity == 0U))
        {
            return false;
        }

        const int length = std::snprintf(
            buffer_out,
            capacity,
            "%s %ld %ld %ld",
            name,
            static_cast<long>(values[0]),
            static_cast<long>(values[1]),
            static_cast<long>(values[2]));

        return (length > 0) && (static_cast<std::size_t>(length) < capacity);
    }

    inline bool format_axis3_i16_line(const char *name, const std::int16_t values[3], char *buffer_out, std::size_t capacity)
    {
        if ((name == nullptr) || (values == nullptr) || (buffer_out == nullptr) || (capacity == 0U))
        {
            return false;
        }

        const int length = std::snprintf(
            buffer_out,
            capacity,
            "%s %d %d %d",
            name,
            static_cast<int>(values[0]),
            static_cast<int>(values[1]),
            static_cast<int>(values[2]));

        return (length > 0) && (static_cast<std::size_t>(length) < capacity);
    }
}
