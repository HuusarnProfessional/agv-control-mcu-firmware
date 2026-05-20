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
    constexpr std::uint32_t sensorfusion_hold_last_valid_ms = 1500U;
    constexpr std::uint16_t mission_seed_heading_confidence = 700U;
    constexpr std::int32_t pi_urad = 3141593;
    constexpr std::int32_t two_pi_urad = 6283185;

    bool previous_mission_running = false;
    bool bypass_offset_fusion = false;
    bool local_only_mode = false;
    bool global_anchor_test_mode = false;
    bool filtered_global_only_mode = true;
    bool has_filtered_global_only_seed_heading = false;
    std::int32_t filtered_global_only_seed_heading_urad = 0;
    bool has_filtered_global_only_heading_rotation = false;
    std::int32_t filtered_global_only_heading_rotation_urad = 0;
    bool has_last_valid_output = false;
    std::uint32_t last_valid_output_time_ms = 0U;
    position_sensorfusion::output_snapshot last_valid_output = {};

    std::uint32_t get_elapsed_ms(std::uint32_t now_ms, std::uint32_t previous_ms)
    {
        if (now_ms < previous_ms)
        {
            return 0U;
        }

        return now_ms - previous_ms;
    }

    void send_branch_request_if_needed(const global_reference_selector::output_snapshot &reference_decision)
    {
        if (reference_decision.request.has_request == false)
        {
            return;
        }

        (void)position_correction_payload::send(reference_decision.request.pose_id, reference_decision.request.branch_id);
    }

    std::int32_t normalize_angle_urad(std::int32_t angle_urad)
    {
        while (angle_urad > pi_urad)
        {
            angle_urad -= two_pi_urad;
        }

        while (angle_urad < -pi_urad)
        {
            angle_urad += two_pi_urad;
        }

        return angle_urad;
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

    void reset_filtered_global_only_heading()
    {
        has_filtered_global_only_seed_heading = false;
        filtered_global_only_seed_heading_urad = 0;
        has_filtered_global_only_heading_rotation = false;
        filtered_global_only_heading_rotation_urad = 0;
    }

    void update_filtered_global_only_seed_heading(const motion_mcu_incoming_state::local_position_state &local_position)
    {
        if (has_filtered_global_only_seed_heading == true)
        {
            if ((has_filtered_global_only_heading_rotation == false) && (local_position.has_pose == true))
            {
                has_filtered_global_only_heading_rotation = true;
                filtered_global_only_heading_rotation_urad = normalize_angle_urad(filtered_global_only_seed_heading_urad - local_position.heading_urad);
            }

            return;
        }

        if (mission_runner::is_running() == false)
        {
            return;
        }

        std::int32_t seed_heading_urad = 0;

        if (mission_reference_seed::read_seed_heading(seed_heading_urad) == false)
        {
            return;
        }

        has_filtered_global_only_seed_heading = true;
        filtered_global_only_seed_heading_urad = seed_heading_urad;

        if (local_position.has_pose == true)
        {
            has_filtered_global_only_heading_rotation = true;
            filtered_global_only_heading_rotation_urad = normalize_angle_urad(filtered_global_only_seed_heading_urad - local_position.heading_urad);
        }
    }

    position_sensorfusion::output_snapshot convert_filtered_global_output(const filtered_global_position::output_snapshot &filtered_position, const motion_mcu_incoming_state::local_position_state &local_position)
    {
        position_sensorfusion::output_snapshot output = {};

        if (filtered_position.has_position == false)
        {
            return output;
        }

        output.has_pose = true;
        output.x_um = filtered_position.x_um;
        output.y_um = filtered_position.y_um;
        output.confidence_position = filtered_position.confidence_position;
        output.pose_id = 0U;
        output.branch_id = 0U;

        if ((has_filtered_global_only_heading_rotation == true) && (local_position.has_pose == true))
        {
            output.heading_urad = normalize_angle_urad(local_position.heading_urad + filtered_global_only_heading_rotation_urad);
            output.confidence_heading = local_position.confidence_heading;
            return output;
        }

        if (has_filtered_global_only_seed_heading == true)
        {
            output.heading_urad = filtered_global_only_seed_heading_urad;
            output.confidence_heading = mission_seed_heading_confidence;
        }

        return output;
    }

    global_reference_selector::current_reference_snapshot convert_current_reference(const motion_mcu_incoming_state::local_position_state &local_position, const local_to_global_transform::output_snapshot &transformed_local_position)
    {
        global_reference_selector::current_reference_snapshot reference = {};

        if (transformed_local_position.has_transform == true)
        {
            if (transformed_local_position.branch_matches == true)
            {
                reference.has_reference = true;
                reference.has_heading = transformed_local_position.has_pose;
                reference.x_um = transformed_local_position.x_um;
                reference.y_um = transformed_local_position.y_um;
                reference.local_x_um = local_position.x_um;
                reference.local_y_um = local_position.y_um;
                reference.heading_urad = transformed_local_position.heading_urad;
                reference.rotation_urad = transformed_local_position.rotation_urad;
            }
        }

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
            reset_filtered_global_only_heading();
            has_last_valid_output = false;
            last_valid_output_time_ms = 0U;
            last_valid_output = {};
        }

        previous_mission_running = mission_running;
    }

    position_sensorfusion::output_snapshot hold_last_valid_output_if_needed(const position_sensorfusion::output_snapshot &output, std::uint32_t now_ms)
    {
        if (output.has_pose == true)
        {
            has_last_valid_output = true;
            last_valid_output_time_ms = now_ms;
            last_valid_output = output;
            return output;
        }

        if (mission_runner::is_running() == true)
        {
            return output;
        }

        if (has_last_valid_output == false)
        {
            return output;
        }

        if (get_elapsed_ms(now_ms, last_valid_output_time_ms) > sensorfusion_hold_last_valid_ms)
        {
            return output;
        }

        return last_valid_output;
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
        filtered_global_only_mode = true;
        reset_filtered_global_only_heading();
        has_last_valid_output = false;
        last_valid_output_time_ms = 0U;
        last_valid_output = {};
    }

    void set_bypass_offset_fusion(bool enabled)
    {
        bypass_offset_fusion = enabled;
        filtered_global_offset_fusion::reset_runtime_state();
        reset_filtered_global_only_heading();
        has_last_valid_output = false;
        last_valid_output_time_ms = 0U;
        last_valid_output = {};
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
            filtered_global_only_mode = false;
        }
        global_reference_selector::reset_runtime_state();
        local_to_global_transform::reset_runtime_state();
        filtered_global_offset_fusion::reset_runtime_state();
        reset_filtered_global_only_heading();
        has_last_valid_output = false;
        last_valid_output_time_ms = 0U;
        last_valid_output = {};
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
            filtered_global_only_mode = false;
        }
        global_reference_selector::reset_runtime_state();
        local_to_global_transform::reset_runtime_state();
        filtered_global_offset_fusion::reset_runtime_state();
        reset_filtered_global_only_heading();
        has_last_valid_output = false;
        last_valid_output_time_ms = 0U;
        last_valid_output = {};
        position_sensorfusion::set_output({});
    }

    bool is_global_anchor_test_mode_enabled()
    {
        return global_anchor_test_mode;
    }

    void set_filtered_global_only_mode(bool enabled)
    {
        filtered_global_only_mode = enabled;
        if (enabled == true)
        {
            local_only_mode = false;
            global_anchor_test_mode = false;
        }
        global_reference_selector::reset_runtime_state();
        local_to_global_transform::reset_runtime_state();
        filtered_global_offset_fusion::reset_runtime_state();
        reset_filtered_global_only_heading();
        has_last_valid_output = false;
        last_valid_output_time_ms = 0U;
        last_valid_output = {};
        position_sensorfusion::set_output({});
    }

    bool is_filtered_global_only_mode_enabled()
    {
        return filtered_global_only_mode;
    }

    void tick(std::uint32_t now_ms)
    {
        update_mission_runtime_state();

        const motion_mcu_incoming_state::local_position_state local_position = motion_mcu_incoming_state::get_local_position();
        filtered_global_position::output_snapshot strong_global_position = filtered_global_position::update(now_ms, local_position);

        if (filtered_global_only_mode == true)
        {
            update_filtered_global_only_seed_heading(local_position);
            position_sensorfusion::set_output(hold_last_valid_output_if_needed(convert_filtered_global_output(strong_global_position, local_position), now_ms));
            return;
        }

        const local_to_global_transform::output_snapshot current_transformed_local_position = local_to_global_transform::read_output(local_position, now_ms);
        const global_reference_selector::current_reference_snapshot current_reference = convert_current_reference(local_position, current_transformed_local_position);
        strong_global_position = filtered_global_position::update_position_anchor_reference(now_ms, current_reference.has_reference, current_transformed_local_position.rotation_urad);
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

        output = hold_last_valid_output_if_needed(convert_output(transformed_local_position), now_ms);

        position_sensorfusion::set_output(output);
    }
}
