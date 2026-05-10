#pragma once

#include <cstddef>
#include <cstdint>

namespace dwm1001_tlv_parser
{

constexpr std::size_t position_response_size = 18u;

enum class parse_status : std::uint8_t
{
    ok = 0u,
    invalid_argument,
    wrong_length,
    missing_return_header,
    request_failed,
    missing_position_header
};

struct position_response
{
    std::int32_t x_mm;
    std::int32_t y_mm;
    std::int32_t z_mm;
    std::uint8_t quality_factor;
};

parse_status parse_position_response(
    const std::uint8_t *data,
    std::size_t length,
    position_response &response_out
);

}
