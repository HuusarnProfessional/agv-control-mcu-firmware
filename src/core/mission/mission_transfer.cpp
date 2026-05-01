#include "mission_transfer.hpp"

#include <cstdio>

namespace
{
    enum class transfer_phase : std::uint8_t
    {
        idle = 0,
        request_part_pending,
        waiting_part_ack,
        waiting_part_info,
        request_chunk_pending,
        waiting_chunk_ack,
        waiting_chunk_data,
        complete,
        failed
    };

    transfer_phase g_transfer_phase = transfer_phase::idle;
    std::uint16_t g_expected_part_number = 0u;
    std::uint16_t g_expected_chunk_number = 0u;

    mission_transfer::transfer_status map_buffer_status(mission_buffer::buffer_status status)
    {
        if (status == mission_buffer::buffer_status::ok)
        {
            return mission_transfer::transfer_status::ok;
        }

        if (status == mission_buffer::buffer_status::invalid_arg)
        {
            return mission_transfer::transfer_status::invalid_arg;
        }

        if (status == mission_buffer::buffer_status::not_found)
        {
            return mission_transfer::transfer_status::not_found;
        }

        if (status == mission_buffer::buffer_status::duplicate)
        {
            return mission_transfer::transfer_status::duplicate;
        }

        if (status == mission_buffer::buffer_status::full)
        {
            return mission_transfer::transfer_status::full;
        }

        if (status == mission_buffer::buffer_status::mismatch)
        {
            return mission_transfer::transfer_status::mission_mismatch;
        }

        return mission_transfer::transfer_status::incomplete;
    }

    void reset_transfer_state()
    {
        g_transfer_phase = transfer_phase::idle;
        g_expected_part_number = 0u;
        g_expected_chunk_number = 0u;
    }

    void advance_to_next_part()
    {
        const std::uint16_t part_count = mission_buffer::part_count();
        const std::uint16_t next_part_number = static_cast<std::uint16_t>(g_expected_part_number + 1u);

        if (next_part_number < part_count)
        {
            g_expected_part_number = next_part_number;
            g_expected_chunk_number = 0u;
            g_transfer_phase = transfer_phase::request_part_pending;
            return;
        }

        g_transfer_phase = transfer_phase::complete;
    }
}

namespace mission_transfer
{
    void init()
    {
        mission_buffer::init();
        reset_transfer_state();
    }

    transfer_status begin_new_mission(const char *mission_id, std::uint16_t number_of_parts)
    {
        const transfer_status status = map_buffer_status(mission_buffer::begin_mission(mission_id, number_of_parts));

        if (status != transfer_status::ok)
        {
            reset_transfer_state();
            return status;
        }

        g_transfer_phase = transfer_phase::request_part_pending;
        g_expected_part_number = 0u;
        g_expected_chunk_number = 0u;

        return transfer_status::ok;
    }

    transfer_status set_part_info(
        const char *mission_id,
        std::uint16_t part_number,
        const char *start_command,
        const char *end_command,
        std::uint16_t path_chunk_count)
    {
        if (mission_buffer::has_mission() == false)
        {
            return transfer_status::no_active_mission;
        }

        if (g_transfer_phase != transfer_phase::waiting_part_info)
        {
            return transfer_status::incomplete;
        }

        if (part_number != g_expected_part_number)
        {
            return transfer_status::invalid_arg;
        }

        const transfer_status status = map_buffer_status(
            mission_buffer::set_part_info(
                mission_id,
                part_number,
                start_command,
                end_command,
                path_chunk_count));

        if (status != transfer_status::ok)
        {
            return status;
        }

        if (path_chunk_count > 0u)
        {
            g_expected_chunk_number = 0u;
            g_transfer_phase = transfer_phase::request_chunk_pending;
            return transfer_status::ok;
        }

        advance_to_next_part();

        return transfer_status::ok;
    }

    transfer_status append_path_chunk(
        const char *mission_id,
        std::uint16_t part_number,
        std::uint16_t chunk_number,
        std::uint16_t point_count,
        const std::uint8_t *path_data,
        std::size_t path_data_length)
    {
        if (mission_buffer::has_mission() == false)
        {
            return transfer_status::no_active_mission;
        }

        if (g_transfer_phase != transfer_phase::waiting_chunk_data)
        {
            return transfer_status::incomplete;
        }

        if ((part_number != g_expected_part_number) || (chunk_number != g_expected_chunk_number))
        {
            return transfer_status::invalid_arg;
        }

        const transfer_status status = map_buffer_status(
            mission_buffer::store_path_chunk(
                mission_id,
                part_number,
                chunk_number,
                point_count,
                path_data,
                path_data_length));

        if (status != transfer_status::ok)
        {
            return status;
        }

        mission_buffer::mission_part_view part_view = {};
        const bool has_part_info = mission_buffer::get_part_info(part_number, part_view);

        if (has_part_info == false)
        {
            return transfer_status::incomplete;
        }

        const std::uint16_t next_chunk_number = static_cast<std::uint16_t>(chunk_number + 1u);

        if (next_chunk_number < part_view.path_chunk_count)
        {
            g_expected_chunk_number = next_chunk_number;
            g_transfer_phase = transfer_phase::request_chunk_pending;
            return transfer_status::ok;
        }

        advance_to_next_part();

        return transfer_status::ok;
    }

    bool has_active_mission()
    {
        return mission_buffer::has_mission();
    }

    bool mission_matches(const char *mission_id)
    {
        return mission_buffer::mission_matches(mission_id);
    }

    bool is_transfer_complete()
    {
        return (g_transfer_phase == transfer_phase::complete) && mission_buffer::is_complete();
    }

    bool has_pending_request()
    {
        return (g_transfer_phase == transfer_phase::request_part_pending) ||
               (g_transfer_phase == transfer_phase::request_chunk_pending);
    }

    bool pop_pending_request(char *request_out, std::size_t capacity)
    {
        if ((request_out == nullptr) || (capacity == 0u))
        {
            return false;
        }

        const char *mission_id = mission_buffer::active_mission_id();

        if (g_transfer_phase == transfer_phase::request_part_pending)
        {
            const int formatted_length = std::snprintf(
                request_out,
                capacity,
                "ini:get_mission_part(%s,%u)",
                mission_id,
                static_cast<unsigned int>(g_expected_part_number));

            if ((formatted_length <= 0) || (static_cast<std::size_t>(formatted_length) >= capacity))
            {
                return false;
            }

            g_transfer_phase = transfer_phase::waiting_part_ack;
            return true;
        }

        if (g_transfer_phase == transfer_phase::request_chunk_pending)
        {
            const int formatted_length = std::snprintf(
                request_out,
                capacity,
                "ini:get_path_chunk(%s,%u,%u)",
                mission_id,
                static_cast<unsigned int>(g_expected_part_number),
                static_cast<unsigned int>(g_expected_chunk_number));

            if ((formatted_length <= 0) || (static_cast<std::size_t>(formatted_length) >= capacity))
            {
                return false;
            }

            g_transfer_phase = transfer_phase::waiting_chunk_ack;
            return true;
        }

        return false;
    }

    void handle_response_ok()
    {
        if (g_transfer_phase == transfer_phase::waiting_part_ack)
        {
            g_transfer_phase = transfer_phase::waiting_part_info;
            return;
        }

        if (g_transfer_phase == transfer_phase::waiting_chunk_ack)
        {
            g_transfer_phase = transfer_phase::waiting_chunk_data;
            return;
        }
    }

    void handle_response_fail(std::uint8_t error_code)
    {
        if ((g_transfer_phase != transfer_phase::waiting_part_ack) &&
            (g_transfer_phase != transfer_phase::waiting_chunk_ack))
        {
            return;
        }

        if (error_code >= 3u)
        {
            g_transfer_phase = transfer_phase::failed;
            return;
        }

        if (g_transfer_phase == transfer_phase::waiting_part_ack)
        {
            g_transfer_phase = transfer_phase::request_part_pending;
            return;
        }

        g_transfer_phase = transfer_phase::request_chunk_pending;
    }

    bool get_part_info(std::uint16_t part_number, mission_buffer::mission_part_view &view_out)
    {
        return mission_buffer::get_part_info(part_number, view_out);
    }

    bool get_path_chunk(std::uint16_t part_number, std::uint16_t chunk_number, mission_buffer::mission_chunk_view &view_out)
    {
        return mission_buffer::get_chunk(part_number, chunk_number, view_out);
    }
}
