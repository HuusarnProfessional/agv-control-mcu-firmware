#include "position_sensorfusion.hpp"

namespace
{
    position_sensorfusion::output_snapshot g_output = {};
}

namespace position_sensorfusion
{
    void set_output(const output_snapshot &snapshot)
    {
        g_output = snapshot;
    }

    output_snapshot read_output()
    {
        return g_output;
    }
}
