#include "mission_pipeline.hpp"

#include "mission_runner.hpp"
#include "mission_transfer.hpp"

namespace mission_pipeline
{
    void init()
    {
        mission_transfer::init();
        mission_runner::init();
    }

    void tick(std::uint32_t now_ms)
    {
        (void)now_ms;
    }
}
