#include "dwm1001_position_link.hpp"

#include "../../platform/esp_uart_api.hpp"

namespace
{

constexpr std::uint32_t request_interval_ms = 100u;
constexpr std::uint32_t response_timeout_ms = 50u;

constexpr std::uint8_t position_get_command[2] = { 0x02u, 0x00u };

enum class link_state : std::uint8_t
{
    idle = 0u,
    waiting_for_response
};

link_state g_state = link_state::idle;

std::uint8_t g_response_buffer[dwm1001_position_link::position_frame_size] = {};
std::size_t g_response_length = 0u;

std::uint32_t g_next_request_id = 1u;
std::uint32_t g_active_request_id = 0u;
std::uint32_t g_request_start_time_ms = 0u;
std::uint32_t g_last_request_finish_time_ms = 0u;

bool g_has_finished_request_once = false;

std::uint32_t allocate_request_id()
{
    const std::uint32_t request_id = g_next_request_id;

    g_next_request_id++;

    if (g_next_request_id == 0u)
    {
        g_next_request_id = 1u;
    }

    return request_id;
}

void reset_response_buffer()
{
    g_response_length = 0u;

    for (std::size_t index = 0u; index < dwm1001_position_link::position_frame_size; index++)
    {
        g_response_buffer[index] = 0u;
    }
}

void reset_active_request(std::uint32_t now_ms)
{
    g_active_request_id = 0u;
    g_last_request_finish_time_ms = now_ms;
    g_has_finished_request_once = true;
    g_state = link_state::idle;

    reset_response_buffer();
}

bool request_interval_has_elapsed(std::uint32_t now_ms)
{
    if (g_has_finished_request_once == false)
    {
        return true;
    }

    const std::uint32_t elapsed_time_ms = now_ms - g_last_request_finish_time_ms;

    if (elapsed_time_ms < request_interval_ms)
    {
        return false;
    }

    return true;
}

void drop_stale_bytes_step()
{
    std::uint8_t drop_buffer[dwm1001_position_link::position_frame_size] = {};

    esp_uart_api::read_bytes(
        esp_uart_api::uart_channel::dwm1001,
        drop_buffer,
        dwm1001_position_link::position_frame_size
    );
}

bool start_position_request(std::uint32_t now_ms)
{
    const esp_uart_api::uart_status write_status = esp_uart_api::write_bytes(
        esp_uart_api::uart_channel::dwm1001,
        position_get_command,
        sizeof(position_get_command)
    );

    if (write_status != esp_uart_api::uart_status::ok)
    {
        return false;
    }

    reset_response_buffer();

    g_active_request_id = allocate_request_id();
    g_request_start_time_ms = now_ms;
    g_state = link_state::waiting_for_response;

    return true;
}

bool copy_completed_frame(std::uint32_t now_ms, dwm1001_position_link::position_frame &frame_out)
{
    frame_out.request_id = g_active_request_id;
    frame_out.received_time_ms = now_ms;
    frame_out.length = g_response_length;

    for (std::size_t index = 0u; index < dwm1001_position_link::position_frame_size; index++)
    {
        frame_out.data[index] = g_response_buffer[index];
    }

    reset_active_request(now_ms);

    return true;
}

void read_available_response_bytes()
{
    const std::size_t remaining_capacity = dwm1001_position_link::position_frame_size - g_response_length;

    if (remaining_capacity == 0u)
    {
        return;
    }

    const std::size_t bytes_read = esp_uart_api::read_bytes(
        esp_uart_api::uart_channel::dwm1001,
        &g_response_buffer[g_response_length],
        remaining_capacity
    );

    if (bytes_read == 0u)
    {
        return;
    }

    g_response_length += bytes_read;
}

bool tick_idle(std::uint32_t now_ms)
{
    if (request_interval_has_elapsed(now_ms) == false)
    {
        return false;
    }

    const std::size_t stale_byte_count = esp_uart_api::available_bytes(
        esp_uart_api::uart_channel::dwm1001
    );

    if (stale_byte_count > 0u)
    {
        drop_stale_bytes_step();
        return false;
    }

    start_position_request(now_ms);

    return false;
}

bool tick_waiting_for_response(std::uint32_t now_ms, dwm1001_position_link::position_frame &frame_out)
{
    read_available_response_bytes();

    if (g_response_length == dwm1001_position_link::position_frame_size)
    {
        return copy_completed_frame(now_ms, frame_out);
    }

    const std::uint32_t elapsed_time_ms = now_ms - g_request_start_time_ms;

    if (elapsed_time_ms >= response_timeout_ms)
    {
        reset_active_request(now_ms);
        return false;
    }

    return false;
}

}

namespace dwm1001_position_link
{

void init()
{
    g_state = link_state::idle;
    g_next_request_id = 1u;
    g_active_request_id = 0u;
    g_request_start_time_ms = 0u;
    g_last_request_finish_time_ms = 0u;
    g_has_finished_request_once = false;

    reset_response_buffer();
}

bool tick(std::uint32_t now_ms, position_frame &frame_out)
{
    if (g_state == link_state::idle)
    {
        return tick_idle(now_ms);
    }

    if (g_state == link_state::waiting_for_response)
    {
        return tick_waiting_for_response(now_ms, frame_out);
    }

    return false;
}

}
