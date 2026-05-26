#include "mission_debug_overrides.hpp"

namespace
{
    bool g_replace_rotate_end_command_with_dummy_enabled = false;
}

namespace mission_debug_overrides
{
    void set_replace_rotate_end_command_with_dummy_enabled(bool enabled)
    {
        g_replace_rotate_end_command_with_dummy_enabled = enabled;
    }

    bool is_replace_rotate_end_command_with_dummy_enabled()
    {
        return g_replace_rotate_end_command_with_dummy_enabled;
    }
}
