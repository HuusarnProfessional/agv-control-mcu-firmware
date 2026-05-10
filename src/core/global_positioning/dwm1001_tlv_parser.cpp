#include "dwm1001_tlv_parser.hpp"

namespace
{

std::uint32_t read_u32_little_endian(const std::uint8_t *data)
{
    const std::uint32_t byte_0 = data[0];
    const std::uint32_t byte_1 = data[1];
    const std::uint32_t byte_2 = data[2];
    const std::uint32_t byte_3 = data[3];

    std::uint32_t value = 0u;

    value |= byte_0;
    value |= byte_1 << 8u;
    value |= byte_2 << 16u;
    value |= byte_3 << 24u;

    return value;
}

std::int32_t read_i32_little_endian(const std::uint8_t *data)
{
    const std::uint32_t value = read_u32_little_endian(data);

    return static_cast<std::int32_t>(value);
}

}

namespace dwm1001_tlv_parser
{

parse_status parse_position_response(const std::uint8_t *data, std::size_t length, position_response &response_out)
{
    if (data == nullptr)
    {
        return parse_status::invalid_argument;
    }

    if (length != position_response_size)
    {
        return parse_status::wrong_length;
    }

    bool return_header_ok = true;

    if (data[0] != 0x40u)
    {
        return_header_ok = false;
    }

    if (data[1] != 0x01u)
    {
        return_header_ok = false;
    }

    if (return_header_ok == false)
    {
        return parse_status::missing_return_header;
    }

    if (data[2] != 0x00u)
    {
        return parse_status::request_failed;
    }

    bool position_header_ok = true;

    if (data[3] != 0x41u)
    {
        position_header_ok = false;
    }

    if (data[4] != 0x0du)
    {
        position_header_ok = false;
    }

    if (position_header_ok == false)
    {
        return parse_status::missing_position_header;
    }

    response_out.x_mm = read_i32_little_endian(&data[5]);
    response_out.y_mm = read_i32_little_endian(&data[9]);
    response_out.z_mm = read_i32_little_endian(&data[13]);
    response_out.quality_factor = data[17];

    return parse_status::ok;
}

}
