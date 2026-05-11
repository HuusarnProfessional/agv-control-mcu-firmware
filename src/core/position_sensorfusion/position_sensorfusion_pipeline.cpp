#include "position_sensorfusion_pipeline.hpp"

#include "global_offset_fusion/global_offset_fusion.hpp"
#include "global_position_heading/global_position_heading.hpp"
#include "global_position_history_filter/global_position_history_filter.hpp"
#include "local_position_alignment_to_global/local_position_alignment_to_global.hpp"
#include "position_sensorfusion.hpp"

#include "../global_positioning/global_position_api.hpp"
#include "../motion_mcu_communication/state/incoming/incoming_state.hpp"

namespace
{
    position_sensorfusion::output_snapshot convert_output(const global_offset_fusion::output_snapshot &offset_output)
    {
        position_sensorfusion::output_snapshot output = {};

        output.has_pose = offset_output.has_pose;
        output.x_um = offset_output.x_um;
        output.y_um = offset_output.y_um;
        output.heading_urad = offset_output.heading_urad;
        output.confidence_position = offset_output.confidence_position;
        output.confidence_heading = offset_output.confidence_heading;
        output.pose_id = offset_output.pose_id;
        output.branch_id = offset_output.branch_id;

        return output;
    }

    global_position_heading::output_snapshot update_global_position_heading()
    {
        global_position_api::global_position_sample global_sample = {};
        const bool has_sample = global_position_api::read_sample(global_sample);

        if (has_sample == true)
        {
            return global_position_heading::update(global_sample);
        }

        return global_position_heading::read_output();
    }
}

namespace position_sensorfusion_pipeline
{
    void init()
    {
        global_position_heading::init();
        global_position_history_filter::init();
        local_position_alignment_to_global::init();
        global_offset_fusion::init();
        position_sensorfusion::set_output({});
    }

    void tick(std::uint32_t now_ms)
    {
        const motion_mcu_incoming_state::local_position_state local_position = motion_mcu_incoming_state::get_local_position();
        const global_position_heading::output_snapshot global_position = update_global_position_heading();
        const global_position_history_filter::output_snapshot filtered_global_position = global_position_history_filter::update(global_position);
        const local_position_alignment_to_global::output_snapshot aligned_local_position = local_position_alignment_to_global::update(local_position, global_position, now_ms);
        const global_offset_fusion::output_snapshot offset_output = global_offset_fusion::update(aligned_local_position, filtered_global_position);
        const position_sensorfusion::output_snapshot output = convert_output(offset_output);

        position_sensorfusion::set_output(output);
    }
}
