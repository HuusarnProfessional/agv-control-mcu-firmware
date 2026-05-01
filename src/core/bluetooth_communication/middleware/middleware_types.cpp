#include "middleware_types.hpp"

namespace middleware_types
{
    void clear_selected_route(selected_route &route)
    {
        route.category = message_category::invalid;

        for (std::size_t index = 0; index < max_command_name_length; ++index)
        {
            route.command_name[index] = '\0';
        }

        route.matched_route = nullptr;
    }
}
