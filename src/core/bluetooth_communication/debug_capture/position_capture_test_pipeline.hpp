#pragma once

#include <cstdint>

namespace position_capture_test_pipeline
{
    void init();

    void tick(std::uint32_t now_ms);

    bool request_run();
}
