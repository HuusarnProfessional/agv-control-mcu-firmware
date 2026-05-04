#pragma once

#include <cstddef>
#include <cstdint>

namespace middleware_parse_helpers
{
    bool discard_until_end(std::uint32_t timeout_us);

    bool read_end(std::uint32_t timeout_us);

    bool read_int16_and_end(std::int16_t &value_out, std::uint32_t timeout_us);

    bool read_uint16_and_end(std::uint16_t &value_out, std::uint32_t timeout_us);

    bool read_uint32_and_end(std::uint32_t &value_out, std::uint32_t timeout_us);

    bool read_int8_and_end(std::int8_t &value_out, std::uint32_t timeout_us);

    bool read_uint8_and_end(std::uint8_t &value_out, std::uint32_t timeout_us);

    bool read_bool_and_end(bool &value_out, std::uint32_t timeout_us);

    bool read_csv_text_and_end(char *buffer_out, std::size_t capacity, std::uint32_t timeout_us);

    bool read_until_comma_or_end(char *buffer_out, std::size_t capacity, bool &ended_out, std::uint32_t timeout_us);

    bool read_command_until_comma_or_end(char *buffer_out, std::size_t capacity, bool &ended_out, std::uint32_t timeout_us);

    bool read_binary(std::uint8_t *buffer_out, std::size_t length, std::uint32_t timeout_us);
}
