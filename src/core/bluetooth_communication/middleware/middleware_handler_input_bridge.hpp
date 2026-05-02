#pragma once

#include <cstdint>

namespace middleware_handler_input_bridge
{
    void clear();

    void set_memory_input(const char *input_text);

    bool read_byte(std::uint8_t &byte_out);
}
