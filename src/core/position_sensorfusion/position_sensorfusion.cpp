#include "position_sensorfusion.hpp"

namespace
{
    position_sensorfusion::output_snapshot latest_output = {};
}

namespace position_sensorfusion
{
    void init()
    {
        latest_output = {};
    }

    void set_output(const output_snapshot &output)
    {
        latest_output = output;
    }

    output_snapshot read_output()
    {
        return latest_output;
    }
}