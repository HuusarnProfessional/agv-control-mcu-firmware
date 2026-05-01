#include "bluetooth_communication_pipeline.hpp"

#include <cstring>

#include "bluetooth_transport.hpp"
#include "middleware/middleware_parser.hpp"
#include "../mission/mission_transfer.hpp"

namespace
{
    middleware_types::selected_route g_selected_route = {};
    bool g_has_selected_route = false;

    bool write_outgoing_request(const char *request_text)
    {
        if (request_text == nullptr)
        {
            return false;
        }

        const std::size_t request_length = std::strlen(request_text);

        return bluetooth_transport::write_bytes(
                   reinterpret_cast<const std::uint8_t *>(request_text),
                   request_length) == bluetooth_transport::transport_status::ok;
    }
}

namespace bluetooth_communication_pipeline
{
    void init()
    {
        middleware_parser::init();
        middleware_types::clear_selected_route(g_selected_route);
        g_has_selected_route = false;
    }

    void tick(std::uint32_t now_ms)
    {
        (void)now_ms;

        if (g_has_selected_route == false)
        {
            std::size_t bytes_processed = 0;

            while ((bluetooth_transport::available_bytes() > 0u) && (bytes_processed < middleware_types::max_message_size))
            {
                std::uint8_t byte_value = 0;

                if (bluetooth_transport::read_byte(byte_value) == false)
                {
                    break;
                }

                ++bytes_processed;

                const middleware_types::parser_status parser_status = middleware_parser::consume_byte(byte_value);

                if (parser_status == middleware_types::parser_status::route_ready)
                {
                    g_has_selected_route = middleware_parser::take_selected_route(g_selected_route);
                    break;
                }

                if ((parser_status == middleware_types::parser_status::parser_error) ||
                    (parser_status == middleware_types::parser_status::message_too_large))
                {
                    middleware_parser::reset();
                    break;
                }
            }
        }

        if (g_has_selected_route == true)
        {
            if ((g_selected_route.matched_route != nullptr) && (g_selected_route.matched_route->handler != nullptr))
            {
                g_selected_route.matched_route->handler();
            }

            middleware_parser::reset();
            middleware_types::clear_selected_route(g_selected_route);
            g_has_selected_route = false;
        }

        if (mission_transfer::has_pending_request() == true)
        {
            char request_text[mission_transfer::max_request_text_length] = {};
            const bool request_ready = mission_transfer::pop_pending_request(request_text, sizeof(request_text));

            if (request_ready == true)
            {
                const bool request_written = write_outgoing_request(request_text);

                if (request_written == false)
                {
                    mission_transfer::handle_response_fail(2u);
                }
            }
        }
    }
}
