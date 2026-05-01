#pragma once

#include <cstddef>
#include <cstdint>

namespace watch_manager
{
    enum class watch_status : std::uint8_t
    {
        ok = 0,
        invalid_arg,
        full,
        duplicate,
        not_found,
        package_too_large
    };

    constexpr std::size_t max_watch_count = 16;
    constexpr std::size_t max_watch_command_length = 96;
    constexpr std::uint16_t max_watch_package_bytes = 990;

    void init();

    void tick(std::uint32_t now_ms);

    watch_status set_keep_alive(std::uint32_t now_ms, std::uint16_t timeout_ms);

    watch_status add_watch(const char *command, std::uint16_t interval_ms, std::uint16_t worst_case_response_bytes, std::uint32_t now_ms);

    watch_status remove_watch(const char *command, std::uint16_t interval_ms);

    bool pop_due_command(std::uint32_t now_ms, char *command_out, std::size_t capacity);

    std::size_t active_count();
}
