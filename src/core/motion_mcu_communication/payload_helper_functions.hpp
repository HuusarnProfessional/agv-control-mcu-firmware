#pragma once

#include <cstddef>
#include <cstdint>

namespace payload_helper_functions
{
    inline bool read_i16_le(const std::uint8_t *data, std::size_t length, std::size_t offset, std::int16_t &value_out)
    {
        if (data == nullptr)
        {
            return false;
        }

        if ((offset + 1U) >= length)
        {
            return false;
        }

        std::uint16_t low = static_cast<std::uint16_t>(data[offset]);
        std::uint16_t high = static_cast<std::uint16_t>(data[offset + 1U]) << 8U;
        value_out = static_cast<std::int16_t>(low | high);
        return true;
    }

    inline bool write_i16_le(std::uint8_t *data, std::size_t length, std::size_t offset, std::int16_t value)
    {
        if (data == nullptr)
        {
            return false;
        }

        if ((offset + 1U) >= length)
        {
            return false;
        }

        std::uint16_t raw_value = static_cast<std::uint16_t>(value);
        data[offset] = static_cast<std::uint8_t>(raw_value & 0x00FFU);
        data[offset + 1U] = static_cast<std::uint8_t>((raw_value >> 8U) & 0x00FFU);
        return true;
    }
}
