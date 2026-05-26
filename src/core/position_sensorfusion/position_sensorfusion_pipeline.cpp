#include "position_sensorfusion_pipeline.hpp"

#include "anchors/anchor_selector.hpp"
#include "anchors/heading_anchor/heading_anchor.hpp"
#include "anchors/position_anchor/position_anchor.hpp"
#include "filtered_global/filtered_global_pipeline.hpp"
#include "seed/mission_reference_seed.hpp"
#include "transform/local_to_global_transform.hpp"
#include "position_sensorfusion.hpp"
#include "internal/geometry_helpers.hpp"
#include "../mission/mission_runner.hpp"
#include "../motion_mcu_communication/state/incoming/incoming_state.hpp"
#include "../motion_mcu_communication/outgoing_payloads/service/position_correction_payload.hpp"
#include "../position_trace/position_trace_logger.hpp"
#include "../control/primitives/motion_primitive/motion_primitive_status_monitor.hpp"

namespace
{
    constexpr std::int64_t entry_seed_x_um = 5250LL * 1000LL;
    constexpr std::int64_t entry_seed_y_um = 1750LL * 1000LL;
    constexpr std::int32_t entry_seed_heading_urad = 3141592;
    bool previous_mission_running = false;
    bool local_only_mode = false;
    bool heading_anchor_enabled = true;
    bool position_anchor_enabled = true;
    bool entry_seeded_drive_test_active = false;
    bool entry_seeded_drive_test_seed_pending = false;

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
            filtered_global_pipeline::init();
            mission_reference_seed::reset_runtime_state();
            anchor_selector::reset_runtime_state();
            local_to_global_transform::reset_runtime_state();
        }

        if ((mission_running == false) && (previous_mission_running == true) && (entry_seeded_drive_test_active == false))
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

        if ((mission_runner::is_running() == true) || (entry_seeded_drive_test_active == true))
        {
            return {};
        }

        return convert_local_position(local_position);
    }

    bool anchors_should_run()
    {
        return mission_runner::is_running() || entry_seeded_drive_test_active;
    }

    const position_sensorfusion_anchors::candidate *find_request_candidate(const position_sensorfusion_anchors::branch_request &request, const position_sensorfusion_anchors::candidate &heading_candidate, const position_sensorfusion_anchors::candidate &position_candidate)
    {
        if ((heading_candidate.valid == true) &&
            (heading_candidate.type == request.type) &&
            (heading_candidate.reference.pose_id == request.pose_id) &&
            (heading_candidate.reference.branch_id == request.branch_id))
        {
            return &heading_candidate;
        }

        if ((position_candidate.valid == true) &&
            (position_candidate.type == request.type) &&
            (position_candidate.reference.pose_id == request.pose_id) &&
            (position_candidate.reference.branch_id == request.branch_id))
        {
            return &position_candidate;
        }

        return nullptr;
    }

    position_trace_logger::anchor_event_snapshot build_trace_event_from_activation(position_trace_logger::event_action action,
                                                                                   const position_sensorfusion_anchors::reference_activation &activation,
                                                                                   const motion_mcu_incoming_state::local_position_state &local_position)
    {
        position_trace_logger::anchor_event_snapshot event = {};

        if (activation.valid == false)
        {
            return event;
        }

        event.valid = true;
        event.action = action;
        event.anchor_type = activation.type;
        event.pose_id = activation.source_pose_id;
        event.branch_id = activation.source_branch_id;
        event.confidence = activation.confidence;
        event.x_um = activation.reference.x_um;
        event.y_um = activation.reference.y_um;
        event.saved_global_heading_urad = activation.saved_global_heading_urad;
        event.activation_local_heading_urad = local_position.heading_urad;

        return event;
    }

    position_trace_logger::anchor_event_snapshot build_trace_event_from_request(const position_sensorfusion_anchors::branch_request &request,
                                                                                const position_sensorfusion_anchors::current_reference &current_reference,
                                                                                const motion_mcu_incoming_state::local_position_state &local_position,
                                                                                const position_sensorfusion_anchors::candidate &heading_candidate,
                                                                                const position_sensorfusion_anchors::candidate &position_candidate)
    {
        position_trace_logger::anchor_event_snapshot event = {};

        if (request.valid == false)
        {
            return event;
        }

        event.valid = true;
        event.action = position_trace_logger::event_action::request;
        event.anchor_type = request.type;
        event.pose_id = request.pose_id;
        event.branch_id = request.branch_id;
        event.confidence = request.confidence;

        const position_sensorfusion_anchors::candidate *candidate = find_request_candidate(request, heading_candidate, position_candidate);

        if (candidate != nullptr)
        {
            event.x_um = candidate->reference.x_um;
            event.y_um = candidate->reference.y_um;
        }

        if (current_reference.has_heading == true)
        {
            event.saved_global_heading_urad = current_reference.heading_urad;
        }

        if (local_position.has_pose == true)
        {
            event.activation_local_heading_urad = local_position.heading_urad;
        }

        return event;
    }

    position_trace_logger::anchor_decision_snapshot build_trace_decision(const anchor_selector::decision_snapshot &decision)
    {
        position_trace_logger::anchor_decision_snapshot trace_decision = {};
        trace_decision.valid = decision.valid;
        trace_decision.selector_state = static_cast<std::uint8_t>(decision.state);
        trace_decision.selected_type = static_cast<std::uint8_t>(decision.selected_type);
        trace_decision.reject_reason = static_cast<std::uint8_t>(decision.reason);
        trace_decision.required_confidence = decision.required_confidence;
        trace_decision.local_confidence = decision.local_confidence;
        trace_decision.heading_confidence = decision.heading_confidence;
        trace_decision.position_confidence = decision.position_confidence;
        trace_decision.heading_pose_id = decision.heading_pose_id;
        trace_decision.position_pose_id = decision.position_pose_id;
        trace_decision.heading_sample_id = decision.heading_sample_id;
        trace_decision.position_sample_id = decision.position_sample_id;
        trace_decision.position_projection_valid = decision.position_projection_valid;
        trace_decision.position_reference_x_um = decision.position_reference_x_um;
        trace_decision.position_reference_y_um = decision.position_reference_y_um;
        trace_decision.position_projected_x_um = decision.position_projected_x_um;
        trace_decision.position_projected_y_um = decision.position_projected_y_um;
        trace_decision.position_jump_x_um = decision.position_jump_x_um;
        trace_decision.position_jump_y_um = decision.position_jump_y_um;
        trace_decision.position_projection_rotation_urad = decision.position_projection_rotation_urad;

        return trace_decision;
    }

    position_sensorfusion_anchors::reference_activation take_entry_seeded_drive_test_activation(const motion_mcu_incoming_state::local_position_state &local_position, std::uint32_t now_ms)
    {
        position_sensorfusion_anchors::reference_activation activation = {};

        if (entry_seeded_drive_test_seed_pending == false)
        {
            return activation;
        }

        if (local_position.has_pose == false)
        {
            return activation;
        }

        activation.valid = true;
        activation.type = position_sensorfusion_anchors::anchor_type::heading_transform;
        activation.is_initial_reference = true;
        activation.is_mission_seed = true;
        activation.source_pose_id = local_position.pose_id;
        activation.source_branch_id = local_position.branch_id;
        activation.activation_time_ms = now_ms;
        activation.confidence = 700U;
        activation.reference.valid = true;
        activation.reference.has_heading = true;
        activation.reference.x_um = entry_seed_x_um;
        activation.reference.y_um = entry_seed_y_um;
        activation.reference.heading_urad = entry_seed_heading_urad;
        activation.reference.confidence_position = 700U;
        activation.reference.confidence_heading = 700U;
        activation.reference.received_time_ms = now_ms;
        entry_seeded_drive_test_seed_pending = false;

        return activation;
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
        position_trace_logger::init();
        position_sensorfusion::set_output({});
        previous_mission_running = false;
        local_only_mode = false;
        heading_anchor_enabled = true;
        position_anchor_enabled = true;
        position_anchor::set_direct_filtered_sample_mode(false);
        anchor_selector::set_position_jump_guard_enabled(true);
        entry_seeded_drive_test_active = false;
        entry_seeded_drive_test_seed_pending = false;
    }

    void set_local_only_mode(bool enabled)
    {
        local_only_mode = enabled;
        anchor_selector::reset_runtime_state();
        local_to_global_transform::reset_runtime_state();
        position_sensorfusion::set_output({});
    }

    bool is_local_only_mode_enabled()
    {
        return local_only_mode;
    }

    void set_heading_anchor_enabled(bool enabled)
    {
        heading_anchor_enabled = enabled;
        anchor_selector::reset_runtime_state();
        heading_anchor::init();
    }

    bool is_heading_anchor_enabled()
    {
        return heading_anchor_enabled;
    }

    void set_position_anchor_enabled(bool enabled)
    {
        position_anchor_enabled = enabled;
        anchor_selector::reset_runtime_state();
        position_anchor::init();
    }

    bool is_position_anchor_enabled()
    {
        return position_anchor_enabled;
    }

    void set_position_anchor_direct_filtered_sample_mode(bool enabled)
    {
        position_anchor::set_direct_filtered_sample_mode(enabled);
        anchor_selector::reset_runtime_state();
    }

    bool is_position_anchor_direct_filtered_sample_mode_enabled()
    {
        return position_anchor::is_direct_filtered_sample_mode_enabled();
    }

    void set_position_anchor_jump_guard_enabled(bool enabled)
    {
        anchor_selector::set_position_jump_guard_enabled(enabled);
        anchor_selector::reset_runtime_state();
    }

    bool is_position_anchor_jump_guard_enabled()
    {
        return anchor_selector::is_position_jump_guard_enabled();
    }

    void start_entry_seeded_drive_forward_test()
    {
        entry_seeded_drive_test_active = true;
        entry_seeded_drive_test_seed_pending = true;
        filtered_global_pipeline::init();
        anchor_selector::reset_runtime_state();
        local_to_global_transform::reset_runtime_state();
        position_sensorfusion::set_output({});
    }

    void tick(std::uint32_t now_ms)
    {
        const motion_mcu_incoming_state::local_position_state local_position = motion_mcu_incoming_state::get_local_position();
        reset_mission_reference_state_if_needed();

        if (entry_seeded_drive_test_active == true)
        {
            const motion_primitive_status_monitor::snapshot motion_status = motion_primitive_status_monitor::read_snapshot();

            if ((motion_status.complete == true) && (motion_status.waiting == false) && (motion_status.running == false))
            {
                entry_seeded_drive_test_active = false;
            }
        }

        const filtered_global::output_snapshot filtered_position = filtered_global_pipeline::tick(now_ms, local_position);

        const local_to_global_transform::output_snapshot current_transformed_position = local_to_global_transform::read_output(local_position);
        const position_sensorfusion_anchors::current_reference current_reference = build_current_reference(local_position, current_transformed_position);
        position_sensorfusion_anchors::candidate heading_candidate = {};
        position_sensorfusion_anchors::candidate position_candidate = {};

        if (heading_anchor_enabled == true)
        {
            heading_candidate = heading_anchor::update();
        }

        if (position_anchor_enabled == true)
        {
            position_candidate = position_anchor::update(current_reference.has_heading, current_reference.rotation_urad);
        }

        const position_sensorfusion_anchors::reference_activation entry_test_seed_activation = take_entry_seeded_drive_test_activation(local_position, now_ms);
        const position_sensorfusion_anchors::reference_activation mission_seed_activation = mission_reference_seed::update(local_position, now_ms);
        anchor_selector::output_snapshot anchor_output = {};

        if ((anchors_should_run() == true) && (local_only_mode == false))
        {
            anchor_output = anchor_selector::update(local_position, current_reference, heading_candidate, position_candidate, now_ms);
        }

        if ((anchor_output.request.valid == true) && (local_only_mode == false))
        {
            (void)position_correction_payload::send(anchor_output.request.pose_id, anchor_output.request.branch_id);
        }

        position_sensorfusion_anchors::reference_activation activation = anchor_output.activation;

        if (mission_seed_activation.valid == true)
        {
            activation = mission_seed_activation;
        }

        if (entry_test_seed_activation.valid == true)
        {
            activation = entry_test_seed_activation;
        }

        const local_to_global_transform::output_snapshot transformed_position = local_to_global_transform::update(local_position, activation);
        const position_sensorfusion::output_snapshot selected_output = select_output(local_position, transformed_position);
        position_sensorfusion::set_output(selected_output);

        position_trace_logger::anchor_event_snapshot trace_event = {};

        if (entry_test_seed_activation.valid == true)
        {
            trace_event = build_trace_event_from_activation(position_trace_logger::event_action::seed, entry_test_seed_activation, local_position);
        }
        else if (mission_seed_activation.valid == true)
        {
            trace_event = build_trace_event_from_activation(position_trace_logger::event_action::seed, mission_seed_activation, local_position);
        }
        else if (anchor_output.activation.valid == true)
        {
            trace_event = build_trace_event_from_activation(position_trace_logger::event_action::activation, anchor_output.activation, local_position);
        }
        else if (anchor_output.request.valid == true)
        {
            trace_event = build_trace_event_from_request(anchor_output.request, current_reference, local_position, heading_candidate, position_candidate);
        }

        position_trace_logger::tick(now_ms, local_position, filtered_position, selected_output, transformed_position.rotation_urad, trace_event, build_trace_decision(anchor_output.decision));
    }
}
