#include "dwm1001_tlv_parser.hpp"

namespace
{

constexpr std::size_t return_tlv_type_index = 0u;
constexpr std::size_t return_tlv_length_index = 1u;
constexpr std::size_t return_tlv_error_code_index = 2u;

constexpr std::size_t position_tlv_type_index = 3u;
constexpr std::size_t position_tlv_length_index = 4u;

constexpr std::size_t position_x_index = 5u;
constexpr std::size_t position_y_index = 9u;
constexpr std::size_t position_z_index = 13u;
constexpr std::size_t position_quality_factor_index = 17u;

constexpr std::uint8_t return_tlv_type = 0x40u;
constexpr std::uint8_t return_tlv_length = 0x01u;
constexpr std::uint8_t return_error_code_ok = 0x00u;

constexpr std::uint8_t position_tlv_type = 0x41u;
constexpr std::uint8_t position_tlv_length = 0x0du;

std::int32_t read_i32_little_endian(const std::uint8_t *data)
{
    const std::uint32_t byte_0 = data[0];
    const std::uint32_t byte_1 = data[1];
    const std::uint32_t byte_2 = data[2];
    const std::uint32_t byte_3 = data[3];

    std::uint32_t unsigned_value = 0u;

    unsigned_value |= byte_0;
    unsigned_value |= byte_1 << 8u;
    unsigned_value |= byte_2 << 16u;
    unsigned_value |= byte_3 << 24u;

    return static_cast<std::int32_t>(unsigned_value);
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

    if (data[return_tlv_type_index] != return_tlv_type)
    {
        return parse_status::missing_return_header;
    }

    if (data[return_tlv_length_index] != return_tlv_length)
    {
        return parse_status::missing_return_header;
    }

    if (data[return_tlv_error_code_index] != return_error_code_ok)
    {
        return parse_status::request_failed;
    }

    if (data[position_tlv_type_index] != position_tlv_type)
    {
        return parse_status::missing_position_header;
    }

    if (data[position_tlv_length_index] != position_tlv_length)
    {
        return parse_status::missing_position_header;
    }

    response_out.x_mm = read_i32_little_endian(&data[position_x_index]);
    response_out.y_mm = read_i32_little_endian(&data[position_y_index]);
    response_out.z_mm = read_i32_little_endian(&data[position_z_index]);
    response_out.quality_factor = data[position_quality_factor_index];

    return parse_status::ok;
}

}
