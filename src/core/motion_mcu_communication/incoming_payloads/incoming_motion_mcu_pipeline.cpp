#include "incoming_motion_mcu_pipeline.hpp"

#include "incoming_payload_definition.hpp"
#include "runtime/local_position_payload.hpp"
#include "runtime/motion_primitive_status_payload.hpp"
#include "runtime/power_status_payload.hpp"
#include "runtime/safety_status_payload.hpp"
#include "../../../platform/esp_uart_api.hpp"
#include "../heartbeat/motion_mcu_heartbeat.hpp"
#include "../motion_mcu_routes.hpp"
#include "../state/incoming/incoming_state.hpp"

namespace
{
    enum class parser_state : std::uint8_t
    {
        wait_for_sync = 0U,
        wait_for_payload_id,
        wait_for_payload_length,
        read_payload_bytes
    };

    parser_state g_parser_state = parser_state::wait_for_sync;
    std::uint8_t g_payload_id = 0U;
    std::uint8_t g_payload_length = 0U;
    std::uint8_t g_payload_index = 0U;
    std::uint8_t g_payload_buffer[motion_mcu_routes::max_payload_length] = {};

    const incoming_payload_definition::route g_routes[] =
    {
        { static_cast<std::uint8_t>(motion_mcu_routes::incoming_payload_id::local_position), local_position_payload::handle },
        { static_cast<std::uint8_t>(motion_mcu_routes::incoming_payload_id::safety_status), safety_status_payload::handle },
        { static_cast<std::uint8_t>(motion_mcu_routes::incoming_payload_id::power_status), power_status_payload::handle },
        { static_cast<std::uint8_t>(motion_mcu_routes::incoming_payload_id::motion_primitive_status), motion_primitive_status_payload::handle }
    };

    void reset_parser()
    {
        g_parser_state = parser_state::wait_for_sync;
        g_payload_id = 0U;
        g_payload_length = 0U;
        g_payload_index = 0U;
    }

    void dispatch_payload(std::uint32_t now_ms)
    {
        for (const incoming_payload_definition::route &route : g_routes)
        {
            if (route.payload_id == g_payload_id)
            {
                if (route.handler != nullptr)
                {
                    motion_mcu_heartbeat::notify_packet_received(now_ms);
                    route.handler(g_payload_buffer, g_payload_length);
                }

                return;
            }
        }
    }

    void process_byte(std::uint8_t byte, std::uint32_t now_ms)
    {
        if (g_parser_state == parser_state::wait_for_sync)
        {
            if (byte == motion_mcu_routes::packet_sync)
            {
                g_parser_state = parser_state::wait_for_payload_id;
            }

            return;
        }

        if (g_parser_state == parser_state::wait_for_payload_id)
        {
            g_payload_id = byte;
            g_parser_state = parser_state::wait_for_payload_length;
            return;
        }

        if (g_parser_state == parser_state::wait_for_payload_length)
        {
            g_payload_length = byte;
            g_payload_index = 0U;

            if (g_payload_length > motion_mcu_routes::max_payload_length)
            {
                reset_parser();
                return;
            }

            if (g_payload_length == 0U)
            {
                dispatch_payload(now_ms);
                reset_parser();
                return;
            }

            g_parser_state = parser_state::read_payload_bytes;
            return;
        }

        if (g_parser_state == parser_state::read_payload_bytes)
        {
            g_payload_buffer[g_payload_index] = byte;
            g_payload_index++;

            if (g_payload_index >= g_payload_length)
            {
                dispatch_payload(now_ms);
                reset_parser();
            }

            return;
        }
    }
}

namespace incoming_motion_mcu_pipeline
{
    void init()
    {
        reset_parser();
        motion_mcu_incoming_state::init();
        motion_mcu_heartbeat::init();
    }

    void tick(std::uint32_t now_ms)
    {
        motion_mcu_heartbeat::tick(now_ms);

        std::uint8_t read_buffer[32u] = {};
        std::size_t read_count = esp_uart_api::read_bytes(
            esp_uart_api::uart_channel::motion_mcu,
            read_buffer,
            sizeof(read_buffer));

        for (std::size_t index = 0u; index < read_count; index++)
        {
            process_byte(read_buffer[index], now_ms);
        }
    }
}
