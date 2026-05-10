#pragma once

#include <cstdint>

namespace global_position_api
{

enum class global_position_status : std::uint8_t
{
    ok = 0u,
    no_sample,
    invalid_response,
    device_error
};

struct global_position_sample
{
    bool valid;
    std::uint32_t sample_id;
    std::uint32_t request_id;
    std::uint32_t received_time_ms;
    std::int32_t x_mm;
    std::int32_t y_mm;
    std::int32_t z_mm;
    std::uint8_t quality_factor;
    global_position_status status;
};

struct global_position_publish_data
{
    std::uint32_t request_id;
    std::uint32_t received_time_ms;
    std::int32_t x_mm;
    std::int32_t y_mm;
    std::int32_t z_mm;
    std::uint8_t quality_factor;
};

void init();

void publish_sample(const global_position_publish_data &publish_data);

bool read_sample(global_position_sample &sample_out);

}
