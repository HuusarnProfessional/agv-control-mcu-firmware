#include "position_sensorfusion_pipeline.hpp"

#include "filtered_global_offset_fusion/filtered_global_offset_fusion.hpp"
#include "filtered_global_position/filtered_global_position.hpp"
#include "global_reference_selector/global_reference_selector.hpp"
#include "local_to_global_transform/local_to_global_transform.hpp"
#include "mission_reference_seed/mission_reference_seed.hpp"
#include "position_sensorfusion.hpp"

#include "../mission/mission_runner.hpp"
#include "../motion_mcu_communication/state/incoming/incoming_state.hpp"
#include "../motion_mcu_communication/outgoing_payloads/service/position_correction_payload.hpp"

namespace
{
    bool previous_mission_running = false;
    bool bypass_offset_fusion = false;
    bool local_only_mode = false;
    bool global_anchor_test_mode = false;

    void send_branch_request_if_needed(const global_reference_selector::output_snapshot &reference_decision)
    {
        if (reference_decision.request.has_request == false)
        {
            return;
        }

        (void)position_correction_payload::send(reference_decision.request.pose_id, reference_decision.request.branch_id);
    }

    position_sensorfusion::output_snapshot convert_output(const local_to_global_transform::output_snapshot &transformed_local_position)
    {
        position_sensorfusion::output_snapshot output = {};

        output.has_pose = transformed_local_position.has_pose;
        output.x_um = transformed_local_position.x_um;
        output.y_um = transformed_local_position.y_um;
        output.heading_urad = transformed_local_position.heading_urad;
        output.confidence_position = transformed_local_position.confidence_position;
        output.confidence_heading = transformed_local_position.confidence_heading;
        output.pose_id = transformed_local_position.pose_id;
        output.branch_id = transformed_local_position.branch_id;

        return output;
    }

    global_reference_selector::current_reference_snapshot convert_current_reference(const local_to_global_transform::output_snapshot &transformed_local_position)
    {
        global_reference_selector::current_reference_snapshot reference = {};

        if (transformed_local_position.has_transform == true)
        {
            if (transformed_local_position.branch_matches == true)
            {
                reference.has_reference = true;
            }
        }

        reference.confidence_position = transformed_local_position.confidence_position;
        reference.confidence_heading = transformed_local_position.confidence_heading;
        reference.branch_id = transformed_local_position.branch_id;

        return reference;
    }

    void update_mission_runtime_state()
    {
        const bool mission_running = mission_runner::is_running();

        if ((mission_running == true) && (previous_mission_running == false))
        {
            mission_reference_seed::reset_runtime_state();
            global_reference_selector::reset_runtime_state();
            local_to_global_transform::reset_runtime_state();
            filtered_global_offset_fusion::reset_runtime_state();
        }

        previous_mission_running = mission_running;
    }
}

namespace position_sensorfusion_pipeline
{
    void init()
    {
        filtered_global_position::init();
        global_reference_selector::init();
        local_to_global_transform::init();
        mission_reference_seed::init();
        filtered_global_offset_fusion::init();
        position_sensorfusion::set_output({});
        previous_mission_running = false;
        bypass_offset_fusion = false;
        local_only_mode = false;
        global_anchor_test_mode = false;
    }

    void set_bypass_offset_fusion(bool enabled)
    {
        bypass_offset_fusion = enabled;
        filtered_global_offset_fusion::reset_runtime_state();
    }

    bool is_bypass_offset_fusion_enabled()
    {
        return bypass_offset_fusion;
    }

    void set_local_only_mode(bool enabled)
    {
        local_only_mode = enabled;
        if (enabled == true)
        {
            global_anchor_test_mode = false;
        }
        global_reference_selector::reset_runtime_state();
        local_to_global_transform::reset_runtime_state();
        filtered_global_offset_fusion::reset_runtime_state();
        position_sensorfusion::set_output({});
    }

    bool is_local_only_mode_enabled()
    {
        return local_only_mode;
    }

    void set_global_anchor_test_mode(bool enabled)
    {
        global_anchor_test_mode = enabled;
        if (enabled == true)
        {
            local_only_mode = false;
        }
        global_reference_selector::reset_runtime_state();
        local_to_global_transform::reset_runtime_state();
        filtered_global_offset_fusion::reset_runtime_state();
        position_sensorfusion::set_output({});
    }

    bool is_global_anchor_test_mode_enabled()
    {
        return global_anchor_test_mode;
    }

    void tick(std::uint32_t now_ms)
    {
        update_mission_runtime_state();

        const motion_mcu_incoming_state::local_position_state local_position = motion_mcu_incoming_state::get_local_position();
        const filtered_global_position::output_snapshot strong_global_position = filtered_global_position::update(now_ms, local_position);
        const local_to_global_transform::output_snapshot current_transformed_local_position = local_to_global_transform::read_output(local_position, now_ms);
        const global_reference_selector::current_reference_snapshot current_reference = convert_current_reference(current_transformed_local_position);
        const global_reference_selector::reference_activation mission_activation = mission_reference_seed::update(local_position, current_reference, now_ms);
        global_reference_selector::output_snapshot reference_decision = {};
        local_to_global_transform::output_snapshot transformed_local_position = {};

        if (mission_activation.has_activation == true)
        {
            transformed_local_position = local_to_global_transform::update(local_position, mission_activation, now_ms);
        }
        else if (local_only_mode == true)
        {
            transformed_local_position = local_to_global_transform::read_output(local_position, now_ms);
        }
        else if (global_anchor_test_mode == true)
        {
            if (mission_runner::is_running() == true)
            {
                reference_decision = global_reference_selector::update(local_position, strong_global_position, current_reference, now_ms);
                transformed_local_position = local_to_global_transform::update(local_position, reference_decision.activation, now_ms);
            }
            else
            {
                transformed_local_position = local_to_global_transform::read_output(local_position, now_ms);
            }
        }
        else if (mission_reference_seed::is_pending() == true)
        {
            transformed_local_position = local_to_global_transform::read_output(local_position, now_ms);
        }
        else if (mission_runner::is_running() == true)
        {
            reference_decision = global_reference_selector::update(local_position, strong_global_position, current_reference, now_ms);
            transformed_local_position = local_to_global_transform::update(local_position, reference_decision.activation, now_ms);
        }
        else
        {
            reference_decision = global_reference_selector::update(local_position, strong_global_position, current_reference, now_ms);
            transformed_local_position = local_to_global_transform::update(local_position, reference_decision.activation, now_ms);
        }

        if (local_only_mode == false)
        {
            send_branch_request_if_needed(reference_decision);
        }

        position_sensorfusion::output_snapshot output = {};

        output = convert_output(transformed_local_position);

        position_sensorfusion::set_output(output);
    }
}
