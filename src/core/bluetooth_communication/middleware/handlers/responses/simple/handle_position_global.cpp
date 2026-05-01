#include "../response_handler_declarations.hpp"

#include "../../../middleware_parse_helpers.hpp"
#include "../../handler_helpers.hpp"

namespace response_handlers
{
    bool handle_position_global()
    {
        char token_buffer[16] = {};
        bool ended = false;
        std::int16_t x = 0;
        std::int16_t y = 0;
        std::int8_t confidence = 0;

        if (middleware_parse_helpers::read_until_comma_or_end(token_buffer, sizeof(token_buffer), ended, 50000u) == false)
        {
            return false;
        }

        if ((ended == true) || (handler_helpers::parse_int16(token_buffer, x) == false))
        {
            return false;
        }

        if (middleware_parse_helpers::read_until_comma_or_end(token_buffer, sizeof(token_buffer), ended, 50000u) == false)
        {
            return false;
        }

        if ((ended == true) || (handler_helpers::parse_int16(token_buffer, y) == false))
        {
            return false;
        }

        if (middleware_parse_helpers::read_until_comma_or_end(token_buffer, sizeof(token_buffer), ended, 50000u) == false)
        {
            return false;
        }

        return (ended == true) && handler_helpers::parse_int8(token_buffer, confidence);
    }
}
