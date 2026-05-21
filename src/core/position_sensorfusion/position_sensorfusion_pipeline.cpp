#include "position_sensorfusion_pipeline.hpp"

#include "anchors/anchor_selector.hpp"
#include "anchors/heading_anchor/heading_anchor.hpp"
#include "anchors/position_anchor/position_anchor.hpp"
#include "filtered_global/filtered_global_pipeline.hpp"
#include "seed/mission_reference_seed.hpp"
#include "transform/local_to_global_transform.hpp"
#include "position_sensorfusion.hpp"
#include "../mission/mission_runner.hpp"
#include "../motion_mcu_communication/state/incoming/incoming_state.hpp"
#include "../motion_mcu_communication/outgoing_payloads/service/position_correction_payload.hpp"

namespace
{
    bool previous_mission_running = false;

    position_sensorfusion::output_snapshot convert_local_position(const motion_mcu_incoming_state::local_position_state &local_position)
    {
        position_sensorfusion::output_snapshot output = {};
        output.has_pose = local_position.has_pose;
        output.x_um = local_position.x_um;
        output.y_um = local_position.y_um;
        output.heading_urad = local_position.heading_urad;
        output.confidence_position = local_position.confidence_position;
        output.confidence_heading = local_position.confidence_heading;
        output.pose_id = local_position.pose_id;
        output.branch_id = local_position.branch_id;

        return output;
    }

    position_sensorfusion::output_snapshot convert_transformed_position(const local_to_global_transform::output_snapshot &transformed_position)
    {
        position_sensorfusion::output_snapshot output = {};
        output.has_pose = transformed_position.has_pose;
        output.x_um = transformed_position.x_um;
        output.y_um = transformed_position.y_um;
        output.heading_urad = transformed_position.heading_urad;
        output.confidence_position = transformed_position.confidence_position;
        output.confidence_heading = transformed_position.confidence_heading;
        output.pose_id = transformed_position.pose_id;
        output.branch_id = transformed_position.branch_id;

        return output;
    }

    position_sensorfusion_anchors::current_reference build_current_reference(const motion_mcu_incoming_state::local_position_state &local_position, const local_to_global_transform::output_snapshot &transformed_position)
    {
        position_sensorfusion_anchors::current_reference reference = {};
        reference.branch_id = transformed_position.branch_id;

        if (transformed_position.has_pose == false)
        {
            return reference;
        }

        reference.valid = true;
        reference.has_heading = true;
        reference.x_um = transformed_position.x_um;
        reference.y_um = transformed_position.y_um;
        reference.local_x_um = local_position.x_um;
        reference.local_y_um = local_position.y_um;
        reference.heading_urad = transformed_position.heading_urad;
        reference.rotation_urad = transformed_position.rotation_urad;

        return reference;
    }

    void reset_mission_reference_state_if_needed()
    {
        const bool mission_running = mission_runner::is_running();

        if ((mission_running == true) && (previous_mission_running == false))
        {
            mission_reference_seed::reset_runtime_state();
            anchor_selector::reset_runtime_state();
            local_to_global_transform::reset_runtime_state();
        }

        if ((mission_running == false) && (previous_mission_running == true))
        {
            anchor_selector::reset_runtime_state();
        }

        previous_mission_running = mission_running;
    }

    position_sensorfusion::output_snapshot select_output(const motion_mcu_incoming_state::local_position_state &local_position, const local_to_global_transform::output_snapshot &transformed_position)
    {
        if (transformed_position.has_pose == true)
        {
            return convert_transformed_position(transformed_position);
        }

        if (mission_runner::is_running() == true)
        {
            return {};
        }

        return convert_local_position(local_position);
    }
}

namespace position_sensorfusion_pipeline
{
    void init()
    {
        filtered_global_pipeline::init();
        heading_anchor::init();
        position_anchor::init();
        anchor_selector::init();
        mission_reference_seed::init();
        local_to_global_transform::init();
        position_sensorfusion::set_output({});
        previous_mission_running = false;
    }

    void tick(std::uint32_t now_ms)
    {
        const motion_mcu_incoming_state::local_position_state local_position = motion_mcu_incoming_state::get_local_position();
        reset_mission_reference_state_if_needed();

        (void)filtered_global_pipeline::tick(now_ms, local_position);

        const local_to_global_transform::output_snapshot current_transformed_position = local_to_global_transform::read_output(local_position);
        const position_sensorfusion_anchors::current_reference current_reference = build_current_reference(local_position, current_transformed_position);
        const position_sensorfusion_anchors::candidate heading_candidate = heading_anchor::update();
        const position_sensorfusion_anchors::candidate position_candidate = position_anchor::update(current_reference.has_heading, current_reference.rotation_urad);
        const position_sensorfusion_anchors::reference_activation seed_activation = mission_reference_seed::update(local_position, now_ms);
        anchor_selector::output_snapshot anchor_output = {};

        if (mission_runner::is_running() == true)
        {
            anchor_output = anchor_selector::update(local_position, current_reference, heading_candidate, position_candidate, now_ms);
        }

        if (anchor_output.request.valid == true)
        {
            (void)position_correction_payload::send(anchor_output.request.pose_id, anchor_output.request.branch_id);
        }

        position_sensorfusion_anchors::reference_activation activation = anchor_output.activation;

        if (seed_activation.valid == true)
        {
            activation = seed_activation;
        }

        const local_to_global_transform::output_snapshot transformed_position = local_to_global_transform::update(local_position, activation);
        position_sensorfusion::set_output(select_output(local_position, transformed_position));
    }
}
