#pragma once

#include <cstddef>
#include <cstdint>

namespace payload_helper_functions
{
    inline bool read_bool(const std::uint8_t *data, std::size_t length, std::size_t offset, bool &value_out)
    {
        if (data == nullptr)
        {
            return false;
        }

        if (offset >= length)
        {
            return false;
        }

        value_out = (data[offset] != 0U);
        return true;
    }

    inline bool read_u8(const std::uint8_t *data, std::size_t length, std::size_t offset, std::uint8_t &value_out)
    {
        if (data == nullptr)
        {
            return false;
        }

        if (offset >= length)
        {
            return false;
        }

        value_out = data[offset];
        return true;
    }

    inline bool read_u16_le(const std::uint8_t *data, std::size_t length, std::size_t offset, std::uint16_t &value_out)
    {
        if (data == nullptr)
        {
            return false;
        }

        if ((offset + 1U) >= length)
        {
            return false;
        }

        const std::uint16_t low = static_cast<std::uint16_t>(data[offset]);
        const std::uint16_t high = static_cast<std::uint16_t>(data[offset + 1U]) << 8U;
        value_out = static_cast<std::uint16_t>(low | high);
        return true;
    }

    inline bool read_i16_le(const std::uint8_t *data, std::size_t length, std::size_t offset, std::int16_t &value_out)
    {
        std::uint16_t raw_value = 0U;

        if (read_u16_le(data, length, offset, raw_value) == false)
        {
            return false;
        }

        value_out = static_cast<std::int16_t>(raw_value);
        return true;
    }

    inline bool read_u32_le(const std::uint8_t *data, std::size_t length, std::size_t offset, std::uint32_t &value_out)
    {
        if (data == nullptr)
        {
            return false;
        }

        if ((offset + 3U) >= length)
        {
            return false;
        }

        value_out =
            static_cast<std::uint32_t>(data[offset]) |
            (static_cast<std::uint32_t>(data[offset + 1U]) << 8U) |
            (static_cast<std::uint32_t>(data[offset + 2U]) << 16U) |
            (static_cast<std::uint32_t>(data[offset + 3U]) << 24U);
        return true;
    }

    inline bool read_i32_le(const std::uint8_t *data, std::size_t length, std::size_t offset, std::int32_t &value_out)
    {
        std::uint32_t raw_value = 0U;

        if (read_u32_le(data, length, offset, raw_value) == false)
        {
            return false;
        }

        value_out = static_cast<std::int32_t>(raw_value);
        return true;
    }

    inline bool read_i64_le(const std::uint8_t *data, std::size_t length, std::size_t offset, std::int64_t &value_out)
    {
        if (data == nullptr)
        {
            return false;
        }

        if ((offset + 7U) >= length)
        {
            return false;
        }

        std::uint64_t raw_value = 0ULL;

        for (std::size_t byte_index = 0U; byte_index < 8U; byte_index++)
        {
            raw_value |= static_cast<std::uint64_t>(data[offset + byte_index]) << (byte_index * 8U);
        }

        value_out = static_cast<std::int64_t>(raw_value);
        return true;
    }

    inline bool write_bool(std::uint8_t *data, std::size_t length, std::size_t offset, bool value)
    {
        if (data == nullptr)
        {
            return false;
        }

        if (offset >= length)
        {
            return false;
        }

        data[offset] = value ? 1U : 0U;
        return true;
    }

    inline bool write_u8(std::uint8_t *data, std::size_t length, std::size_t offset, std::uint8_t value)
    {
        if (data == nullptr)
        {
            return false;
        }

        if (offset >= length)
        {
            return false;
        }

        data[offset] = value;
        return true;
    }

    inline bool write_u16_le(std::uint8_t *data, std::size_t length, std::size_t offset, std::uint16_t value)
    {
        if (data == nullptr)
        {
            return false;
        }

        if ((offset + 1U) >= length)
        {
            return false;
        }

        data[offset] = static_cast<std::uint8_t>(value & 0x00FFU);
        data[offset + 1U] = static_cast<std::uint8_t>((value >> 8U) & 0x00FFU);
        return true;
    }

    inline bool write_i16_le(std::uint8_t *data, std::size_t length, std::size_t offset, std::int16_t value)
    {
        return write_u16_le(data, length, offset, static_cast<std::uint16_t>(value));
    }

    inline bool write_u32_le(std::uint8_t *data, std::size_t length, std::size_t offset, std::uint32_t value)
    {
        if (data == nullptr)
        {
            return false;
        }

        if ((offset + 3U) >= length)
        {
            return false;
        }

        data[offset] = static_cast<std::uint8_t>(value & 0x000000FFUL);
        data[offset + 1U] = static_cast<std::uint8_t>((value >> 8U) & 0x000000FFUL);
        data[offset + 2U] = static_cast<std::uint8_t>((value >> 16U) & 0x000000FFUL);
        data[offset + 3U] = static_cast<std::uint8_t>((value >> 24U) & 0x000000FFUL);
        return true;
    }

    inline bool write_i32_le(std::uint8_t *data, std::size_t length, std::size_t offset, std::int32_t value)
    {
        return write_u32_le(data, length, offset, static_cast<std::uint32_t>(value));
    }

    inline bool write_i64_le(std::uint8_t *data, std::size_t length, std::size_t offset, std::int64_t value)
    {
        if (data == nullptr)
        {
            return false;
        }

        if ((offset + 7U) >= length)
        {
            return false;
        }

        const std::uint64_t raw_value = static_cast<std::uint64_t>(value);

        for (std::size_t byte_index = 0U; byte_index < 8U; byte_index++)
        {
            data[offset + byte_index] = static_cast<std::uint8_t>((raw_value >> (byte_index * 8U)) & 0xFFULL);
        }

        return true;
    }
}
