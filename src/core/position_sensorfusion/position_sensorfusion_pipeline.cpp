#include "position_sensorfusion_pipeline.hpp"

#include "position_sensorfusion.hpp"
#include "../motion_mcu_communication/state/incoming/incoming_state.hpp"

namespace position_sensorfusion_pipeline
{
    void init()
    {
        position_sensorfusion::set_output({});
    }

    void tick(std::uint32_t now_ms)
    {
        (void)now_ms;

        const motion_mcu_incoming_state::local_position_state local_position = motion_mcu_incoming_state::get_local_position();
        position_sensorfusion::output_snapshot output = {};
        output.has_pose = local_position.has_pose;
        output.x_um = local_position.x_um;
        output.y_um = local_position.y_um;
        output.heading_urad = local_position.heading_urad;
        output.confidence_position = local_position.confidence_position;
        output.confidence_heading = local_position.confidence_heading;
        output.pose_id = local_position.pose_id;
        output.branch_id = local_position.branch_id;
        position_sensorfusion::set_output(output);
    }
}
