#include "mission_buffer.hpp"

namespace
{
    struct mission_part_record
    {
        bool is_set;
        mission_buffer::mission_command_view start_command;
        mission_buffer::mission_command_view end_command;
        std::uint16_t path_chunk_count;
    };

    struct mission_chunk_record
    {
        bool is_set;
        std::uint16_t part_number;
        std::uint16_t chunk_number;
        std::uint16_t point_count;
        std::size_t data_offset;
        std::size_t data_length;
    };

    char g_mission_id[mission_buffer::max_mission_id_length] = {};
    std::uint16_t g_part_count = 0;
    bool g_has_mission = false;
    mission_part_record g_parts[mission_buffer::max_part_count] = {};
    mission_chunk_record g_chunks[mission_buffer::max_chunk_count] = {};
    std::uint8_t g_path_data[mission_buffer::max_path_data_bytes] = {};
    std::size_t g_used_path_data_bytes = 0;

    void clear_text_buffer(char *buffer_out, std::size_t capacity)
    {
        if ((buffer_out == nullptr) || (capacity == 0u))
        {
            return;
        }

        for (std::size_t index = 0; index < capacity; ++index)
        {
            buffer_out[index] = '\0';
        }
    }

    bool strings_equal(const char *left, const char *right)
    {
        if ((left == nullptr) || (right == nullptr))
        {
            return false;
        }

        std::size_t index = 0u;

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

    bool copy_text(char *destination, std::size_t capacity, const char *source)
    {
        if ((destination == nullptr) || (capacity == 0u) || (source == nullptr))
        {
            return false;
        }

        std::size_t index = 0u;

        while (source[index] != '\0')
        {
            if ((index + 1u) >= capacity)
            {
                return false;
            }

            destination[index] = source[index];
            ++index;
        }

        destination[index] = '\0';
        return true;
    }

    void clear_command_view(mission_buffer::mission_command_view &command_view)
    {
        command_view.route = nullptr;
        clear_text_buffer(command_view.argument_stream, sizeof(command_view.argument_stream));
    }

    bool copy_command_view(
        mission_buffer::mission_command_view &destination,
        const mission_buffer::mission_command_view &source)
    {
        if (source.route == nullptr)
        {
            return false;
        }

        destination.route = source.route;
        return copy_text(destination.argument_stream, sizeof(destination.argument_stream), source.argument_stream);
    }

    void clear_parts()
    {
        for (std::size_t index = 0u; index < mission_buffer::max_part_count; ++index)
        {
            g_parts[index].is_set = false;
            clear_command_view(g_parts[index].start_command);
            clear_command_view(g_parts[index].end_command);
            g_parts[index].path_chunk_count = 0u;
        }
    }

    void clear_chunks()
    {
        for (std::size_t index = 0u; index < mission_buffer::max_chunk_count; ++index)
        {
            g_chunks[index].is_set = false;
            g_chunks[index].part_number = 0u;
            g_chunks[index].chunk_number = 0u;
            g_chunks[index].point_count = 0u;
            g_chunks[index].data_offset = 0u;
            g_chunks[index].data_length = 0u;
        }

        for (std::size_t index = 0u; index < mission_buffer::max_path_data_bytes; ++index)
        {
            g_path_data[index] = 0u;
        }

        g_used_path_data_bytes = 0u;
    }

    bool chunk_exists(std::uint16_t part_number, std::uint16_t chunk_number)
    {
        for (std::size_t index = 0u; index < mission_buffer::max_chunk_count; ++index)
        {
            if ((g_chunks[index].is_set == true) &&
                (g_chunks[index].part_number == part_number) &&
                (g_chunks[index].chunk_number == chunk_number))
            {
                return true;
            }
        }

        return false;
    }

    mission_chunk_record *find_free_chunk_slot()
    {
        for (std::size_t index = 0u; index < mission_buffer::max_chunk_count; ++index)
        {
            if (g_chunks[index].is_set == false)
            {
                return &g_chunks[index];
            }
        }

        return nullptr;
    }

    bool is_valid_part_number(std::uint16_t part_number)
    {
        if (g_has_mission == false)
        {
            return false;
        }

        return part_number < g_part_count;
    }
}

namespace mission_buffer
{
    void init()
    {
        clear();
    }

    buffer_status clear()
    {
        clear_text_buffer(g_mission_id, sizeof(g_mission_id));
        g_part_count = 0u;
        g_has_mission = false;
        clear_parts();
        clear_chunks();

        return buffer_status::ok;
    }

    buffer_status begin_mission(const char *mission_id, std::uint16_t number_of_parts)
    {
        if ((mission_id == nullptr) || (number_of_parts == 0u) || (number_of_parts > max_part_count))
        {
            return buffer_status::invalid_arg;
        }

        clear();

        if (copy_text(g_mission_id, sizeof(g_mission_id), mission_id) == false)
        {
            return buffer_status::invalid_arg;
        }

        g_part_count = number_of_parts;
        g_has_mission = true;

        return buffer_status::ok;
    }

    bool has_mission()
    {
        return g_has_mission;
    }

    bool mission_matches(const char *mission_id)
    {
        if (g_has_mission == false)
        {
            return false;
        }

        return strings_equal(g_mission_id, mission_id);
    }

    const char *active_mission_id()
    {
        return g_mission_id;
    }

    std::uint16_t part_count()
    {
        return g_part_count;
    }

    buffer_status set_part_info(
        const char *mission_id,
        std::uint16_t part_number,
        const mission_command_view &start_command,
        const mission_command_view &end_command,
        std::uint16_t path_chunk_count)
    {
        if ((start_command.route == nullptr) || (end_command.route == nullptr) || (path_chunk_count > max_chunk_count))
        {
            return buffer_status::invalid_arg;
        }

        if (mission_matches(mission_id) == false)
        {
            return buffer_status::mismatch;
        }

        if (is_valid_part_number(part_number) == false)
        {
            return buffer_status::not_found;
        }

        mission_part_record &part_record = g_parts[part_number];

        if (part_record.is_set == true)
        {
            return buffer_status::duplicate;
        }

        if (copy_command_view(part_record.start_command, start_command) == false)
        {
            return buffer_status::invalid_arg;
        }

        if (copy_command_view(part_record.end_command, end_command) == false)
        {
            return buffer_status::invalid_arg;
        }

        part_record.path_chunk_count = path_chunk_count;
        part_record.is_set = true;

        return buffer_status::ok;
    }

    buffer_status store_path_chunk(
        const char *mission_id,
        std::uint16_t part_number,
        std::uint16_t chunk_number,
        std::uint16_t point_count,
        const std::uint8_t *path_data,
        std::size_t path_data_length)
    {
        if (((path_data == nullptr) && (path_data_length > 0u)) ||
            (path_data_length > max_chunk_data_length) ||
            (path_data_length != (static_cast<std::size_t>(point_count) * 4u)))
        {
            return buffer_status::invalid_arg;
        }

        if (mission_matches(mission_id) == false)
        {
            return buffer_status::mismatch;
        }

        if (is_valid_part_number(part_number) == false)
        {
            return buffer_status::not_found;
        }

        mission_part_record &part_record = g_parts[part_number];

        if (part_record.is_set == false)
        {
            return buffer_status::incomplete;
        }

        if (chunk_number >= part_record.path_chunk_count)
        {
            return buffer_status::invalid_arg;
        }

        if (chunk_exists(part_number, chunk_number) == true)
        {
            return buffer_status::duplicate;
        }

        if ((g_used_path_data_bytes + path_data_length) > max_path_data_bytes)
        {
            return buffer_status::full;
        }

        mission_chunk_record *chunk_slot = find_free_chunk_slot();

        if (chunk_slot == nullptr)
        {
            return buffer_status::full;
        }

        const std::size_t data_offset = g_used_path_data_bytes;

        for (std::size_t index = 0u; index < path_data_length; ++index)
        {
            g_path_data[data_offset + index] = path_data[index];
        }

        chunk_slot->is_set = true;
        chunk_slot->part_number = part_number;
        chunk_slot->chunk_number = chunk_number;
        chunk_slot->point_count = point_count;
        chunk_slot->data_offset = data_offset;
        chunk_slot->data_length = path_data_length;

        g_used_path_data_bytes += path_data_length;

        return buffer_status::ok;
    }

    bool get_part_info(std::uint16_t part_number, mission_part_view &view_out)
    {
        if ((g_has_mission == false) || (part_number >= g_part_count))
        {
            return false;
        }

        const mission_part_record &part_record = g_parts[part_number];

        if (part_record.is_set == false)
        {
            return false;
        }

        if (copy_command_view(view_out.start_command, part_record.start_command) == false)
        {
            return false;
        }

        if (copy_command_view(view_out.end_command, part_record.end_command) == false)
        {
            return false;
        }

        view_out.path_chunk_count = part_record.path_chunk_count;

        return true;
    }

    bool get_chunk(std::uint16_t part_number, std::uint16_t chunk_number, mission_chunk_view &view_out)
    {
        for (std::size_t index = 0u; index < max_chunk_count; ++index)
        {
            const mission_chunk_record &chunk_record = g_chunks[index];

            if ((chunk_record.is_set == true) &&
                (chunk_record.part_number == part_number) &&
                (chunk_record.chunk_number == chunk_number))
            {
                view_out.point_count = chunk_record.point_count;
                view_out.data = &g_path_data[chunk_record.data_offset];
                view_out.data_length = chunk_record.data_length;

                return true;
            }
        }

        return false;
    }

    bool is_complete()
    {
        if (g_has_mission == false)
        {
            return false;
        }

        for (std::uint16_t part_number = 0u; part_number < g_part_count; ++part_number)
        {
            const mission_part_record &part_record = g_parts[part_number];

            if (part_record.is_set == false)
            {
                return false;
            }

            for (std::uint16_t chunk_number = 0u; chunk_number < part_record.path_chunk_count; ++chunk_number)
            {
                if (chunk_exists(part_number, chunk_number) == false)
                {
                    return false;
                }
            }
        }

        return true;
    }
}
