#include "path_accessor.hpp"

#include "../pure_pursuit_tuning.hpp"
#include "../../mission/mission_buffer.hpp"

namespace
{
    std::int16_t read_point_int16_little_endian(const std::uint8_t *data, std::size_t offset)
    {
        const std::uint16_t low = static_cast<std::uint16_t>(data[offset]);
        const std::uint16_t high = static_cast<std::uint16_t>(data[offset + 1u]) << 8u;
        return static_cast<std::int16_t>(low | high);
    }
}

namespace pure_pursuit_internal
{
    bool get_point_count(std::uint16_t part_number, std::uint16_t &point_count_out)
    {
        mission_buffer::mission_part_view part_view = {};
        const bool has_part_info = mission_buffer::get_part_info(part_number, part_view);

        if (has_part_info == false)
        {
            return false;
        }

        if (part_view.path_chunk_count > pure_pursuit_tuning::k_mission_max_chunk_iterations)
        {
            return false;
        }

        std::uint32_t total_point_count = 0u;

        for (std::uint16_t chunk_number = 0u; chunk_number < part_view.path_chunk_count; ++chunk_number)
        {
            mission_buffer::mission_chunk_view chunk_view = {};
            const bool has_chunk = mission_buffer::get_chunk(part_number, chunk_number, chunk_view);

            if (has_chunk == false)
            {
                return false;
            }

            total_point_count += chunk_view.point_count;

            if (total_point_count > 65535u)
            {
                return false;
            }
        }

        point_count_out = static_cast<std::uint16_t>(total_point_count);
        return true;
    }

    bool get_point(std::uint16_t part_number, std::uint16_t point_index, path_point &point_out)
    {
        mission_buffer::mission_part_view part_view = {};
        const bool has_part_info = mission_buffer::get_part_info(part_number, part_view);

        if (has_part_info == false)
        {
            return false;
        }

        if (part_view.path_chunk_count > pure_pursuit_tuning::k_mission_max_chunk_iterations)
        {
            return false;
        }

        std::uint32_t point_base_index = 0u;

        for (std::uint16_t chunk_number = 0u; chunk_number < part_view.path_chunk_count; ++chunk_number)
        {
            mission_buffer::mission_chunk_view chunk_view = {};
            const bool has_chunk = mission_buffer::get_chunk(part_number, chunk_number, chunk_view);

            if (has_chunk == false)
            {
                return false;
            }

            const std::uint32_t chunk_end_index = point_base_index + chunk_view.point_count;

            if (static_cast<std::uint32_t>(point_index) < chunk_end_index)
            {
                const std::uint32_t point_index_in_chunk = static_cast<std::uint32_t>(point_index) - point_base_index;
                const std::size_t point_offset = static_cast<std::size_t>(point_index_in_chunk) * 4u;

                if ((point_offset + 3u) >= chunk_view.data_length)
                {
                    return false;
                }

                point_out.x_mm = read_point_int16_little_endian(chunk_view.data, point_offset);
                point_out.y_mm = read_point_int16_little_endian(chunk_view.data, point_offset + 2u);
                return true;
            }

            point_base_index = chunk_end_index;
        }

        return false;
    }
}
