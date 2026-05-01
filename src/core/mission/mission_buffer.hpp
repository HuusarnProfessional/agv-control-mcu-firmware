#pragma once

#include <cstddef>
#include <cstdint>

namespace mission_buffer
{
    constexpr std::size_t max_mission_id_length = 64u;
    constexpr std::size_t max_command_length = 64u;
    constexpr std::uint16_t max_part_count = 16u;
    constexpr std::uint16_t max_chunk_count = 64u;
    constexpr std::size_t max_chunk_data_length = 990u;
    constexpr std::size_t max_path_data_bytes = 8192u;

    enum class buffer_status : std::uint8_t
    {
        ok = 0,
        invalid_arg,
        not_found,
        duplicate,
        full,
        mismatch,
        incomplete
    };

    struct mission_part_view
    {
        char start_command[max_command_length];
        char end_command[max_command_length];
        std::uint16_t path_chunk_count;
    };

    struct mission_chunk_view
    {
        std::uint16_t point_count;
        const std::uint8_t *data;
        std::size_t data_length;
    };

    void init();

    buffer_status clear();

    buffer_status begin_mission(const char *mission_id, std::uint16_t number_of_parts);

    bool has_mission();

    bool mission_matches(const char *mission_id);

    const char *active_mission_id();

    std::uint16_t part_count();

    buffer_status set_part_info(
        const char *mission_id,
        std::uint16_t part_number,
        const char *start_command,
        const char *end_command,
        std::uint16_t path_chunk_count);

    buffer_status store_path_chunk(
        const char *mission_id,
        std::uint16_t part_number,
        std::uint16_t chunk_number,
        std::uint16_t point_count,
        const std::uint8_t *path_data,
        std::size_t path_data_length);

    bool get_part_info(std::uint16_t part_number, mission_part_view &view_out);

    bool get_chunk(std::uint16_t part_number, std::uint16_t chunk_number, mission_chunk_view &view_out);

    bool is_complete();
}
