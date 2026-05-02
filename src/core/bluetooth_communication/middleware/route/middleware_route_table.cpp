#include "middleware_route_table.hpp"

#include "../handlers/incoming_requests/incoming_request_handler_declarations.hpp"
#include "../handlers/initiated_requests/initiated_request_handler_declarations.hpp"
#include "../handlers/responses/response_handler_declarations.hpp"

namespace
{
    const middleware_route_types::middleware_command_route g_empty_routes[] =
    {
    };

    const middleware_route_types::middleware_command_route g_incoming_request_routes[] =
    {
        {"ping()", incoming_request_handlers::handle_ping},
        {"reset()", incoming_request_handlers::handle_reset},
        {"get_speed()", incoming_request_handlers::handle_get_speed},
        {"get_position_local()", incoming_request_handlers::handle_get_position_local},
        {"get_position_global()", incoming_request_handlers::handle_get_position_global},
        {"set_armed(bool:is_armed)", incoming_request_handlers::handle_set_armed},
        {"get_armed()", incoming_request_handlers::handle_get_armed},
        {"set_drive_forward_mm(int16_t:distance_mm)", incoming_request_handlers::handle_set_drive_forward_mm},
        {"set_mission_new(const char:mission_id,uint16_t:number_of_parts)", incoming_request_handlers::handle_set_mission_new},
        {"mission_part_info(const char:mission_id,uint16_t:part_number,const char:start_command,const char:end_command,uint16_t:path_chunk_count)", incoming_request_handlers::handle_mission_part_info},
        {"path_chunk(const char:mission_id,uint16_t:part_number,uint16_t:chunk_number,uint16_t:point_count,const uint8_t:path_data[point_count * 4])", incoming_request_handlers::handle_path_chunk},
        {"set_mission_start(const char:mission_id)", incoming_request_handlers::handle_set_mission_start},
        {"set_mission_abort()", incoming_request_handlers::handle_set_mission_abort},
        {"set_pause_ms(uint32_t duration_ms)", incoming_request_handlers::handle_set_pause_ms},
        {"set_speed(uint16_t:speed_mm_per_s)", incoming_request_handlers::handle_set_speed},
        {"get_mission_part_current()", incoming_request_handlers::handle_get_mission_part_current},
        {"set_drive_rotate_deg(int16_t:delta_heading)", incoming_request_handlers::handle_set_drive_rotate_deg},
        {"set_watch_keep_alive(uint16_t:timeout_ms)", incoming_request_handlers::handle_set_watch_keep_alive},
        {"set_watch_add(const char:command,uint16_t:interval_ms)", incoming_request_handlers::handle_set_watch_add},
        {"set_watch_remove(const char:command,uint16_t:interval_ms)", incoming_request_handlers::handle_set_watch_remove},
        {"dummy()", incoming_request_handlers::handle_dummy}
    };

    const middleware_route_types::middleware_command_route g_initiated_request_routes[] =
    {
        {"get_mission_part(const char:mission_id,uint16_t:part_number)", initiated_request_handlers::handle_get_mission_part},
        {"get_path_chunk(const char:mission_id,uint16_t:part_number,uint16_t:chunk_number)", initiated_request_handlers::handle_get_path_chunk}
    };

    const middleware_route_types::middleware_command_route g_response_routes[] =
    {
        {"pong()", response_handlers::handle_pong},
        {"ok()", response_handlers::handle_ok},
        {"fail(uint8_t:error_code)", response_handlers::handle_fail},
        {"speed(int16_t:speed_mm_per_s)", response_handlers::handle_speed},
        {"mission_part_current(uint16_t:part_number)", response_handlers::handle_mission_part_current},
        {"armed(bool:is_armed)", response_handlers::handle_armed}
    };

    const middleware_route_types::middleware_route_table g_empty_route_table =
    {
        g_empty_routes,
        0
    };

    const middleware_route_types::middleware_route_table g_incoming_request_route_table =
    {
        g_incoming_request_routes,
        sizeof(g_incoming_request_routes) / sizeof(g_incoming_request_routes[0])
    };

    const middleware_route_types::middleware_route_table g_initiated_request_route_table =
    {
        g_initiated_request_routes,
        sizeof(g_initiated_request_routes) / sizeof(g_initiated_request_routes[0])
    };

    const middleware_route_types::middleware_route_table g_response_route_table =
    {
        g_response_routes,
        sizeof(g_response_routes) / sizeof(g_response_routes[0])
    };

    bool command_pattern_matches(const char *command_pattern, const char *command_name)
    {
        if ((command_pattern == nullptr) || (command_name == nullptr))
        {
            return false;
        }

        std::size_t index = 0u;

        while ((command_pattern[index] != '\0') && (command_pattern[index] != '(') && (command_name[index] != '\0'))
        {
            if (command_pattern[index] != command_name[index])
            {
                return false;
            }

            ++index;
        }

        return ((command_pattern[index] == '(') || (command_pattern[index] == '\0')) && (command_name[index] == '\0');
    }
}

namespace middleware_route_table
{
    const middleware_route_types::middleware_route_table &get_table(middleware_types::message_category category)
    {
        if (category == middleware_types::message_category::incoming_request)
        {
            return g_incoming_request_route_table;
        }

        if (category == middleware_types::message_category::initiated_request)
        {
            return g_initiated_request_route_table;
        }

        if (category == middleware_types::message_category::response)
        {
            return g_response_route_table;
        }

        return g_empty_route_table;
    }

    const middleware_route_types::middleware_route_table &incoming_request_routes()
    {
        return g_incoming_request_route_table;
    }

    const middleware_route_types::middleware_route_table &initiated_request_routes()
    {
        return g_initiated_request_route_table;
    }

    const middleware_route_types::middleware_route_table &response_routes()
    {
        return g_response_route_table;
    }

    const middleware_route_types::middleware_command_route *find_incoming_request_route(const char *command_name)
    {
        for (std::size_t index = 0u; index < g_incoming_request_route_table.route_count; ++index)
        {
            const middleware_route_types::middleware_command_route &route = g_incoming_request_route_table.routes[index];

            if (command_pattern_matches(route.command_pattern, command_name) == true)
            {
                return &route;
            }
        }

        return nullptr;
    }
}
