#include "global_positioning_pipeline.hpp"

#include "dwm1001_position_link.hpp"
#include "dwm1001_tlv_parser.hpp"
#include "global_position_api.hpp"

namespace
{

void publish_parsed_response(
    const dwm1001_position_link::position_frame &frame,
    const dwm1001_tlv_parser::position_response &response
)
{
    global_position_api::global_position_publish_data publish_data = {};

    publish_data.request_id = frame.request_id;
    publish_data.received_time_ms = frame.received_time_ms;
    publish_data.x_mm = response.x_mm;
    publish_data.y_mm = response.y_mm;
    publish_data.z_mm = response.z_mm;
    publish_data.quality_factor = response.quality_factor;

    global_position_api::publish_sample(publish_data);
}

void handle_position_frame(const dwm1001_position_link::position_frame &frame)
{
    dwm1001_tlv_parser::position_response response = {};

    const dwm1001_tlv_parser::parse_status status =
        dwm1001_tlv_parser::parse_position_response(
            frame.data,
            frame.length,
            response
        );

    if (status != dwm1001_tlv_parser::parse_status::ok)
    {
        return;
    }

    publish_parsed_response(frame, response);
}

}

namespace global_positioning_pipeline
{

void init()
{
    global_position_api::init();
    dwm1001_position_link::init();
}

void tick(std::uint32_t now_ms)
{
    dwm1001_position_link::position_frame frame = {};

    const bool has_frame = dwm1001_position_link::tick(now_ms, frame);

    if (has_frame == false)
    {
        return;
    }

    handle_position_frame(frame);
}
}
