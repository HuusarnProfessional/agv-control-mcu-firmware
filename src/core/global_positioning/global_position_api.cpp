#include "global_position_api.hpp"

namespace
{

global_position_api::global_position_sample g_latest_sample = {};

std::uint32_t g_next_sample_id = 1u;

std::uint32_t allocate_sample_id()
{
    const std::uint32_t sample_id = g_next_sample_id;

    g_next_sample_id++;

    if (g_next_sample_id == 0u)
    {
        g_next_sample_id = 1u;
    }

    return sample_id;
}

}

namespace global_position_api
{

void init()
{
    g_latest_sample.valid = false;
    g_latest_sample.sample_id = 0u;
    g_latest_sample.request_id = 0u;
    g_latest_sample.received_time_ms = 0u;
    g_latest_sample.x_mm = 0;
    g_latest_sample.y_mm = 0;
    g_latest_sample.z_mm = 0;
    g_latest_sample.quality_factor = 0u;
    g_latest_sample.status = global_position_status::no_sample;

    g_next_sample_id = 1u;
}

void publish_sample(const global_position_publish_data &publish_data)
{
    g_latest_sample.valid = true;
    g_latest_sample.sample_id = allocate_sample_id();
    g_latest_sample.request_id = publish_data.request_id;
    g_latest_sample.received_time_ms = publish_data.received_time_ms;
    g_latest_sample.x_mm = publish_data.x_mm;
    g_latest_sample.y_mm = publish_data.y_mm;
    g_latest_sample.z_mm = publish_data.z_mm;
    g_latest_sample.quality_factor = publish_data.quality_factor;
    g_latest_sample.status = global_position_status::ok;
}

bool read_sample(global_position_sample &sample_out)
{
    if (g_latest_sample.valid == false)
    {
        return false;
    }

    sample_out = g_latest_sample;

    return true;
}

}
