#pragma once

#include <cstddef>
#include <cstdint>

namespace middleware_route_types
{
    struct middleware_command_route;
}

namespace middleware_types
{
    constexpr std::size_t max_message_size = 990;
    constexpr std::size_t max_category_length = 3;
    constexpr std::size_t max_command_name_length = 64;

    enum class message_category : std::uint8_t
    {
        invalid = 0,
        incoming_request,
        initiated_request,
        response
    };

    enum class parser_status : std::uint8_t
    {
        idle = 0,
        in_progress,
        route_ready,
        parser_error,
        message_too_large
    };

    enum class middleware_error_code : std::uint8_t
    {
        unknown_error = 0,
        parser_error = 1,
        invalid_argument = 2,
        mission_transfer_error = 3,
        control_system_error = 4,
        drive_system_error = 5,
        collision_detected = 6
    };

    struct selected_route
    {
        message_category category;
        char command_name[max_command_name_length];
        const middleware_route_types::middleware_command_route *matched_route;
    };

    void clear_selected_route(selected_route &route);
}
