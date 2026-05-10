#pragma once

#include <cstddef>
#include <cstdint>

namespace dwm1001_position_link
{

constexpr std::size_t position_frame_size = 18u;

struct position_frame
{
    std::uint32_t request_id;
    std::uint32_t received_time_ms;
    std::uint8_t data[position_frame_size];
    std::size_t length;
};

void init();

bool tick(std::uint32_t now_ms, position_frame &frame_out);

}
