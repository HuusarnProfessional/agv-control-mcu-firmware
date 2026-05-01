#pragma once

#include <cstddef>

namespace middleware_route_types
{
    using middleware_handler = bool (*)();

    struct middleware_command_route
    {
        const char *command_pattern;
        middleware_handler handler;
    };

    struct middleware_route_table
    {
        const middleware_command_route *routes;
        std::size_t route_count;
    };
}
