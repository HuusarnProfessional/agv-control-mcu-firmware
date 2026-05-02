#pragma once

#include "../middleware_types.hpp"
#include "middleware_route_types.hpp"

namespace middleware_route_table
{
    const middleware_route_types::middleware_route_table &get_table(middleware_types::message_category category);

    const middleware_route_types::middleware_route_table &incoming_request_routes();

    const middleware_route_types::middleware_route_table &initiated_request_routes();

    const middleware_route_types::middleware_route_table &response_routes();

    const middleware_route_types::middleware_command_route *find_incoming_request_route(const char *command_name);
}
