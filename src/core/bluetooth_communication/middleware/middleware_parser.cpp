#include "middleware_parser.hpp"

#include <cstddef>

#include "route/middleware_route_table.hpp"

namespace
{
    enum class parse_phase : std::uint8_t
    {
        read_category = 0,
        read_command_name
    };

    middleware_types::parser_status g_status = middleware_types::parser_status::idle;
    middleware_types::selected_route g_selected_route = {};
    char g_category_buffer[middleware_types::max_category_length + 1] = {};
    std::size_t g_category_length = 0;
    std::size_t g_command_name_length = 0;
    std::size_t g_total_bytes = 0;
    parse_phase g_phase = parse_phase::read_category;

    bool strings_equal(const char *left, const char *right)
    {
        if ((left == nullptr) || (right == nullptr))
        {
            return false;
        }

        std::size_t index = 0;

        while ((left[index] != '\0') && (right[index] != '\0'))
        {
            if (left[index] != right[index])
            {
                return false;
            }

            ++index;
        }

        return (left[index] == '\0') && (right[index] == '\0');
    }

    middleware_types::message_category parse_category()
    {
        if (strings_equal(g_category_buffer, "inc") == true)
        {
            return middleware_types::message_category::incoming_request;
        }

        if (strings_equal(g_category_buffer, "ini") == true)
        {
            return middleware_types::message_category::initiated_request;
        }

        if (strings_equal(g_category_buffer, "rsp") == true)
        {
            return middleware_types::message_category::response;
        }

        return middleware_types::message_category::invalid;
    }

    bool command_pattern_matches(const char *command_pattern, const char *command_name)
    {
        if ((command_pattern == nullptr) || (command_name == nullptr))
        {
            return false;
        }

        std::size_t index = 0;

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

    const middleware_route_types::middleware_command_route *find_matching_route(
        middleware_types::message_category category,
        const char *command_name)
    {
        const middleware_route_types::middleware_route_table &route_table = middleware_route_table::get_table(category);

        for (std::size_t index = 0; index < route_table.route_count; ++index)
        {
            const middleware_route_types::middleware_command_route &route = route_table.routes[index];

            if (command_pattern_matches(route.command_pattern, command_name) == true)
            {
                return &route;
            }
        }

        return nullptr;
    }

    void set_error_status(middleware_types::parser_status next_status)
    {
        g_status = next_status;
        middleware_types::clear_selected_route(g_selected_route);
    }
}

namespace middleware_parser
{
    void init()
    {
        reset();
    }

    middleware_types::parser_status status()
    {
        return g_status;
    }

    middleware_types::parser_status consume_byte(std::uint8_t byte_value)
    {
        if (g_status == middleware_types::parser_status::route_ready)
        {
            return g_status;
        }

        if ((g_status == middleware_types::parser_status::parser_error) ||
            (g_status == middleware_types::parser_status::message_too_large))
        {
            return g_status;
        }

        if (g_total_bytes >= middleware_types::max_message_size)
        {
            set_error_status(middleware_types::parser_status::message_too_large);
            return g_status;
        }

        ++g_total_bytes;
        g_status = middleware_types::parser_status::in_progress;

        if (g_phase == parse_phase::read_category)
        {
            if (byte_value == static_cast<std::uint8_t>(':'))
            {
                if (g_category_length == 0)
                {
                    set_error_status(middleware_types::parser_status::parser_error);
                    return g_status;
                }

                g_selected_route.category = parse_category();

                if (g_selected_route.category == middleware_types::message_category::invalid)
                {
                    set_error_status(middleware_types::parser_status::parser_error);
                    return g_status;
                }

                g_phase = parse_phase::read_command_name;
                return g_status;
            }

            if (g_category_length >= middleware_types::max_category_length)
            {
                set_error_status(middleware_types::parser_status::message_too_large);
                return g_status;
            }

            g_category_buffer[g_category_length] = static_cast<char>(byte_value);
            ++g_category_length;
            g_category_buffer[g_category_length] = '\0';

            return g_status;
        }

        if (byte_value == static_cast<std::uint8_t>('('))
        {
            if (g_command_name_length == 0)
            {
                set_error_status(middleware_types::parser_status::parser_error);
                return g_status;
            }

            const middleware_route_types::middleware_command_route *matched_route =
                find_matching_route(g_selected_route.category, g_selected_route.command_name);

            if (matched_route == nullptr)
            {
                set_error_status(middleware_types::parser_status::parser_error);
                return g_status;
            }

            g_selected_route.matched_route = matched_route;
            g_status = middleware_types::parser_status::route_ready;

            return g_status;
        }

        if (g_command_name_length + 1 >= middleware_types::max_command_name_length)
        {
            set_error_status(middleware_types::parser_status::message_too_large);
            return g_status;
        }

        g_selected_route.command_name[g_command_name_length] = static_cast<char>(byte_value);
        ++g_command_name_length;
        g_selected_route.command_name[g_command_name_length] = '\0';

        return g_status;
    }

    bool take_selected_route(middleware_types::selected_route &route_out)
    {
        if (g_status != middleware_types::parser_status::route_ready)
        {
            return false;
        }

        route_out = g_selected_route;
        return true;
    }

    void reset()
    {
        g_status = middleware_types::parser_status::idle;
        middleware_types::clear_selected_route(g_selected_route);

        for (std::size_t index = 0; index < (middleware_types::max_category_length + 1); ++index)
        {
            g_category_buffer[index] = '\0';
        }

        g_category_length = 0;
        g_command_name_length = 0;
        g_total_bytes = 0;
        g_phase = parse_phase::read_category;
    }
}
