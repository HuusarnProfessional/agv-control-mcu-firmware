#pragma once

#include <cstdint>

#include "middleware_types.hpp"

namespace middleware_parser
{
    void init();

    middleware_types::parser_status status();

    middleware_types::parser_status consume_byte(std::uint8_t byte_value);

    bool take_selected_route(middleware_types::selected_route &route_out);

    void reset();
}
