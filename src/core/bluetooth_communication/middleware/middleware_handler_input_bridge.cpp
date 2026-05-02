#include "middleware_handler_input_bridge.hpp"

#include "../bluetooth_transport.hpp"

namespace
{
    const char *g_memory_input = nullptr;
    std::uint32_t g_memory_index = 0u;
}

namespace middleware_handler_input_bridge
{
    void clear()
    {
        g_memory_input = nullptr;
        g_memory_index = 0u;
    }

    void set_memory_input(const char *input_text)
    {
        g_memory_input = input_text;
        g_memory_index = 0u;
    }

    bool read_byte(std::uint8_t &byte_out)
    {
        if (g_memory_input != nullptr)
        {
            const char next_char = g_memory_input[g_memory_index];

            if (next_char == '\0')
            {
                return false;
            }

            byte_out = static_cast<std::uint8_t>(next_char);
            ++g_memory_index;
            return true;
        }

        return bluetooth_transport::read_byte(byte_out);
    }
}
