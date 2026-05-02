#pragma once

#include <cstddef>
#include <cstdint>

#include "mission_buffer.hpp"

namespace mission_transfer
{
    constexpr std::size_t max_request_text_length = 160u;

    enum class transfer_status : std::uint8_t
    {
        ok = 0,
        invalid_arg,
        no_active_mission,
        mission_mismatch,
        duplicate,
        full,
        incomplete,
        not_found
    };

    void init();

    transfer_status begin_new_mission(const char *mission_id, std::uint16_t number_of_parts);

    transfer_status set_part_info(
        const char *mission_id,
        std::uint16_t part_number,
        const mission_buffer::mission_command_view &start_command,
        const mission_buffer::mission_command_view &end_command,
        std::uint16_t path_chunk_count);

    transfer_status append_path_chunk(
        const char *mission_id,
        std::uint16_t part_number,
        std::uint16_t chunk_number,
        std::uint16_t point_count,
        const std::uint8_t *path_data,
        std::size_t path_data_length);

    bool has_active_mission();

    bool mission_matches(const char *mission_id);

    bool is_transfer_complete();

    bool has_pending_request();

    bool pop_pending_request(char *request_out, std::size_t capacity);

    void handle_response_ok();

    void handle_response_fail(std::uint8_t error_code);

    bool get_part_info(std::uint16_t part_number, mission_buffer::mission_part_view &view_out);

    bool get_path_chunk(std::uint16_t part_number, std::uint16_t chunk_number, mission_buffer::mission_chunk_view &view_out);
}
