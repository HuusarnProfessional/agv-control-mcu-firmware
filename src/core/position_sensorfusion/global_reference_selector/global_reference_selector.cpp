#include "global_reference_selector.hpp"

#include <cstdint>

namespace
{
    constexpr std::uint16_t full_confidence = 1000U;
    constexpr std::uint16_t minimum_anchor_confidence = 300U;
    constexpr std::uint16_t reference_switch_margin_percent = 10U;
    constexpr std::uint16_t pending_switch_margin_percent = 25U;
    constexpr std::uint32_t minimum_request_interval_ms = 1000U;
    constexpr std::uint32_t pending_request_timeout_ms = 3000U;
    constexpr std::uint32_t settling_time_ms = 500U;
    constexpr std::uint16_t maximum_reference_pose_age_steps = 2048U;
    constexpr std::int32_t pi_urad = 3141593;
    constexpr std::int32_t two_pi_urad = 6283185;
    constexpr std::int32_t maximum_anchor_heading_delta_urad = 523599;
    constexpr std::uint8_t request_reason_none = 0U;
    constexpr std::uint8_t request_reason_missing_reference = 1U;
    constexpr std::uint8_t request_reason_switch_margin = 2U;
    constexpr std::uint8_t request_reason_pending_margin = 3U;
    constexpr std::uint8_t request_reason_heading_mismatch = 4U;

    struct pending_reference_state
    {
        bool pending = false;
        std::uint16_t source_pose_id = 0U;
        std::uint8_t source_branch_id = 0U;
        std::uint32_t request_time_ms = 0U;
        std::uint16_t reference_confidence = 0U;
        filtered_global_position::output_snapshot global_reference = {};
    };

    pending_reference_state pending_reference = {};
    bool settling = false;
    std::uint32_t settling_start_time_ms = 0U;
    bool has_last_request = false;
    std::uint32_t last_request_time_ms = 0U;
    std::uint32_t last_request_global_sample_id = 0U;
    std::uint16_t last_request_confidence = 0U;
    global_reference_selector::output_snapshot latest_output = {};

    std::uint32_t get_age_ms(std::uint32_t now_ms, std::uint32_t start_time_ms)
    {
        if (now_ms < start_time_ms)
        {
            return 0U;
        }

        return now_ms - start_time_ms;
    }

    std::uint16_t smaller_confidence(std::uint16_t left, std::uint16_t right)
    {
        if (left < right)
        {
            return left;
        }

        return right;
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

    std::int32_t absolute_angle_delta_urad(std::int32_t left_urad, std::int32_t right_urad)
    {
        const std::int32_t delta_urad = normalize_angle_urad(left_urad - right_urad);

        if (delta_urad < 0)
        {
            return -delta_urad;
        }

        return delta_urad;
    }

    std::uint16_t calculate_margin_confidence(std::uint16_t confidence, std::uint16_t margin_percent)
    {
        const std::uint32_t margin = static_cast<std::uint32_t>(confidence) * static_cast<std::uint32_t>(margin_percent) / 100U;
        const std::uint32_t required_confidence = static_cast<std::uint32_t>(confidence) + margin;

        if (required_confidence > full_confidence)
        {
            return full_confidence;
        }

        return static_cast<std::uint16_t>(required_confidence);
    }

    bool confidence_is_better_by_percent(std::uint16_t new_confidence, std::uint16_t old_confidence, std::uint16_t margin_percent)
    {
        const std::uint16_t required_confidence = calculate_margin_confidence(old_confidence, margin_percent);

        if (new_confidence > required_confidence)
        {
            return true;
        }

        return false;
    }

    bool global_position_can_be_reference(const filtered_global_position::output_snapshot &global_position)
    {
        if (global_position.has_position == false)
        {
            return false;
        }

        if (global_position.has_heading == false)
        {
            return false;
        }

        if (global_position.is_new_sample == false)
        {
            return false;
        }

        if (global_position.accepted == false)
        {
            return false;
        }

        if (global_position.rejected == true)
        {
            return false;
        }

        if (global_position.heading_reference_pose_id == 0U)
        {
            return false;
        }

        if (global_position.heading_reference_branch_id > 1U)
        {
            return false;
        }

        return true;
    }

    std::uint16_t calculate_candidate_anchor_confidence(const filtered_global_position::output_snapshot &global_position)
    {
        if (global_position_can_be_reference(global_position) == false)
        {
            return 0U;
        }

        return global_position.candidate_anchor_confidence;
    }

    std::uint16_t calculate_local_reference_confidence(const motion_mcu_incoming_state::local_position_state &local_position)
    {
        if (local_position.has_pose == false)
        {
            return 0U;
        }

        return smaller_confidence(local_position.confidence_position, local_position.confidence_heading);
    }

    std::int32_t calculate_candidate_anchor_heading_delta_urad(const filtered_global_position::output_snapshot &global_position, const global_reference_selector::current_reference_snapshot &current_reference)
    {
        if (current_reference.has_reference == false)
        {
            return 0;
        }

        if (current_reference.has_heading == false)
        {
            return 0;
        }

        if (global_position.has_heading == false)
        {
            return 0;
        }

        return absolute_angle_delta_urad(global_position.heading_urad, current_reference.heading_urad);
    }

    bool candidate_anchor_heading_is_consistent(std::int32_t heading_delta_urad, const global_reference_selector::current_reference_snapshot &current_reference)
    {
        if (current_reference.has_reference == false)
        {
            return true;
        }

        if (current_reference.has_heading == false)
        {
            return true;
        }

        if (heading_delta_urad <= maximum_anchor_heading_delta_urad)
        {
            return true;
        }

        return false;
    }

    bool normal_request_interval_has_passed(std::uint32_t now_ms)
    {
        if (has_last_request == false)
        {
            return true;
        }

        const std::uint32_t age_ms = get_age_ms(now_ms, last_request_time_ms);

        if (age_ms >= minimum_request_interval_ms)
        {
            return true;
        }

        return false;
    }

    void update_settling(std::uint32_t now_ms)
    {
        if (settling == false)
        {
            return;
        }

        const std::uint32_t age_ms = get_age_ms(now_ms, settling_start_time_ms);

        if (age_ms >= settling_time_ms)
        {
            settling = false;
        }
    }

    void update_pending_timeout(std::uint32_t now_ms)
    {
        if (pending_reference.pending == false)
        {
            return;
        }

        const std::uint32_t age_ms = get_age_ms(now_ms, pending_reference.request_time_ms);

        if (age_ms < pending_request_timeout_ms)
        {
            return;
        }

        pending_reference = {};
    }

    void mark_last_request(const filtered_global_position::output_snapshot &global_position, std::uint16_t reference_confidence, std::uint32_t now_ms)
    {
        has_last_request = true;
        last_request_time_ms = now_ms;
        last_request_global_sample_id = global_position.sample_id;
        last_request_confidence = reference_confidence;
    }

    std::uint16_t calculate_pose_age_steps(std::uint16_t current_pose_id, std::uint16_t reference_pose_id)
    {
        return static_cast<std::uint16_t>(current_pose_id - reference_pose_id);
    }

    bool global_reference_pose_can_be_replayed(const motion_mcu_incoming_state::local_position_state &local_position, const filtered_global_position::output_snapshot &global_position)
    {
        if (global_position.heading_reference_branch_id != local_position.branch_id)
        {
            return false;
        }

        const std::uint16_t pose_age_steps = calculate_pose_age_steps(local_position.pose_id, global_position.heading_reference_pose_id);

        if (pose_age_steps > maximum_reference_pose_age_steps)
        {
            return false;
        }

        return true;
    }

    filtered_global_position::output_snapshot build_anchor_reference(const filtered_global_position::output_snapshot &global_position)
    {
        filtered_global_position::output_snapshot anchor_reference = global_position;

        anchor_reference.x_um = global_position.heading_reference_x_um;
        anchor_reference.y_um = global_position.heading_reference_y_um;
        anchor_reference.z_um = global_position.heading_reference_z_um;
        anchor_reference.sample_id = global_position.heading_reference_sample_id;
        anchor_reference.received_time_ms = global_position.heading_reference_time_ms;

        return anchor_reference;
    }

    global_reference_selector::reference_activation build_initial_activation(const motion_mcu_incoming_state::local_position_state &local_position, const filtered_global_position::output_snapshot &global_position, std::uint16_t reference_confidence, std::uint32_t now_ms)
    {
        global_reference_selector::reference_activation activation = {};

        if (global_position.has_heading == false)
        {
            return activation;
        }

        activation.has_activation = true;
        activation.is_initial_reference = true;
        activation.is_mission_seed = false;
        activation.source_pose_id = local_position.pose_id;
        activation.source_branch_id = local_position.branch_id;
        activation.activation_time_ms = now_ms;
        activation.reference_confidence = reference_confidence;
        activation.global_reference = build_anchor_reference(global_position);

        return activation;
    }

    global_reference_selector::reference_activation build_pending_activation(std::uint32_t now_ms)
    {
        global_reference_selector::reference_activation activation = {};

        activation.has_activation = true;
        activation.is_initial_reference = false;
        activation.is_mission_seed = false;
        activation.source_pose_id = pending_reference.source_pose_id;
        activation.source_branch_id = pending_reference.source_branch_id;
        activation.activation_time_ms = now_ms;
        activation.reference_confidence = pending_reference.reference_confidence;
        activation.global_reference = pending_reference.global_reference;

        pending_reference = {};
        settling = true;
        settling_start_time_ms = now_ms;

        return activation;
    }

    global_reference_selector::branch_request start_pending_request(const motion_mcu_incoming_state::local_position_state &local_position, const filtered_global_position::output_snapshot &global_position, std::uint16_t reference_confidence, std::uint32_t now_ms)
    {
        global_reference_selector::branch_request request = {};

        if (global_position.has_heading == false)
        {
            return request;
        }

        if (global_reference_pose_can_be_replayed(local_position, global_position) == false)
        {
            return request;
        }

        pending_reference.pending = true;
        pending_reference.source_pose_id = global_position.heading_reference_pose_id;
        pending_reference.source_branch_id = global_position.heading_reference_branch_id;
        pending_reference.request_time_ms = now_ms;
        pending_reference.reference_confidence = reference_confidence;
        pending_reference.global_reference = build_anchor_reference(global_position);

        mark_last_request(global_position, reference_confidence, now_ms);

        request.has_request = true;
        request.pose_id = pending_reference.source_pose_id;
        request.branch_id = pending_reference.source_branch_id;
        request.reference_confidence = reference_confidence;

        return request;
    }

    bool branch_has_arrived(const motion_mcu_incoming_state::local_position_state &local_position)
    {
        if (pending_reference.pending == false)
        {
            return false;
        }

        if (local_position.branch_id != pending_reference.source_branch_id)
        {
            return true;
        }

        return false;
    }

    bool request_is_allowed_by_interval(std::uint16_t reference_confidence, std::uint32_t now_ms)
    {
        if (normal_request_interval_has_passed(now_ms) == true)
        {
            return true;
        }

        if (confidence_is_better_by_percent(reference_confidence, last_request_confidence, pending_switch_margin_percent) == true)
        {
            return true;
        }

        return false;
    }

    global_reference_selector::output_snapshot build_output(const motion_mcu_incoming_state::local_position_state &local_position, const filtered_global_position::output_snapshot &global_position, std::uint16_t local_reference_confidence, std::uint16_t candidate_anchor_confidence, std::uint16_t required_anchor_confidence, std::int32_t candidate_anchor_heading_delta_urad, bool candidate_anchor_heading_consistent)
    {
        global_reference_selector::output_snapshot output = {};

        output.pending = pending_reference.pending;
        output.settling = settling;
        output.pending_pose_id = pending_reference.source_pose_id;
        output.pending_branch_id = pending_reference.source_branch_id;
        output.pending_global_sample_id = pending_reference.global_reference.sample_id;
        output.local_position_confidence = local_position.confidence_position;
        output.local_heading_confidence = local_position.confidence_heading;
        output.local_reference_confidence = local_reference_confidence;
        output.candidate_anchor_position_confidence = global_position.candidate_anchor_position_confidence;
        output.candidate_anchor_heading_confidence = global_position.candidate_anchor_heading_confidence;
        output.candidate_anchor_adjusted_heading_confidence = global_position.candidate_anchor_adjusted_heading_confidence;
        output.candidate_anchor_confidence = candidate_anchor_confidence;
        output.candidate_anchor_heading_delta_urad = candidate_anchor_heading_delta_urad;
        output.candidate_anchor_heading_consistent = candidate_anchor_heading_consistent;
        output.required_anchor_confidence = required_anchor_confidence;
        output.request_reason = request_reason_none;

        return output;
    }
}

namespace global_reference_selector
{
    void init()
    {
        reset_runtime_state();
    }

    void reset_runtime_state()
    {
        pending_reference = {};
        settling = false;
        settling_start_time_ms = 0U;
        has_last_request = false;
        last_request_time_ms = 0U;
        last_request_global_sample_id = 0U;
        last_request_confidence = 0U;
        latest_output = {};
    }

    output_snapshot update(const motion_mcu_incoming_state::local_position_state &local_position, const filtered_global_position::output_snapshot &global_position, const current_reference_snapshot &current_reference, std::uint32_t now_ms)
    {
        update_settling(now_ms);
        update_pending_timeout(now_ms);

        const std::uint16_t candidate_anchor_confidence = calculate_candidate_anchor_confidence(global_position);
        const std::uint16_t local_reference_confidence = calculate_local_reference_confidence(local_position);
        const std::uint16_t normal_required_anchor_confidence = calculate_margin_confidence(local_reference_confidence, reference_switch_margin_percent);
        const std::uint16_t pending_required_anchor_confidence = calculate_margin_confidence(pending_reference.reference_confidence, pending_switch_margin_percent);
        const std::int32_t candidate_anchor_heading_delta_urad = calculate_candidate_anchor_heading_delta_urad(global_position, current_reference);
        const bool candidate_anchor_heading_consistent = candidate_anchor_heading_is_consistent(candidate_anchor_heading_delta_urad, current_reference);
        output_snapshot output = build_output(local_position, global_position, local_reference_confidence, candidate_anchor_confidence, normal_required_anchor_confidence, candidate_anchor_heading_delta_urad, candidate_anchor_heading_consistent);

        if (local_position.has_pose == false)
        {
            latest_output = output;
            return latest_output;
        }

        if (branch_has_arrived(local_position) == true)
        {
            output.activation = build_pending_activation(now_ms);
            output.pending = pending_reference.pending;
            output.settling = settling;
            output.pending_pose_id = pending_reference.source_pose_id;
            output.pending_branch_id = pending_reference.source_branch_id;
            output.pending_global_sample_id = pending_reference.global_reference.sample_id;
            latest_output = output;
            return latest_output;
        }

        if (candidate_anchor_confidence < minimum_anchor_confidence)
        {
            latest_output = output;
            return latest_output;
        }

        if (candidate_anchor_heading_consistent == false)
        {
            output.request_reason = request_reason_heading_mismatch;
            latest_output = output;
            return latest_output;
        }

        if (current_reference.has_reference == false)
        {
            if ((global_position.heading_reference_pose_id == local_position.pose_id) && (global_position.heading_reference_branch_id == local_position.branch_id))
            {
                output.activation = build_initial_activation(local_position, global_position, candidate_anchor_confidence, now_ms);
                latest_output = output;
                return latest_output;
            }

            output.request = start_pending_request(local_position, global_position, candidate_anchor_confidence, now_ms);
            output.pending = pending_reference.pending;
            output.pending_pose_id = pending_reference.source_pose_id;
            output.pending_branch_id = pending_reference.source_branch_id;
            output.pending_global_sample_id = pending_reference.global_reference.sample_id;
            if (output.request.has_request == true)
            {
                output.request_reason = request_reason_missing_reference;
            }
            latest_output = output;
            return latest_output;
        }

        if (pending_reference.pending == true)
        {
            output.required_anchor_confidence = pending_required_anchor_confidence;

            if (confidence_is_better_by_percent(candidate_anchor_confidence, pending_reference.reference_confidence, pending_switch_margin_percent) == false)
            {
                latest_output = output;
                return latest_output;
            }

            if (request_is_allowed_by_interval(candidate_anchor_confidence, now_ms) == false)
            {
                latest_output = output;
                return latest_output;
            }

            output.request = start_pending_request(local_position, global_position, candidate_anchor_confidence, now_ms);
            output.pending = pending_reference.pending;
            output.pending_pose_id = pending_reference.source_pose_id;
            output.pending_branch_id = pending_reference.source_branch_id;
            output.pending_global_sample_id = pending_reference.global_reference.sample_id;
            if (output.request.has_request == true)
            {
                output.request_reason = request_reason_pending_margin;
            }
            latest_output = output;
            return latest_output;
        }

        if (settling == true)
        {
            latest_output = output;
            return latest_output;
        }

        if (has_last_request == true)
        {
            if (global_position.sample_id == last_request_global_sample_id)
            {
                latest_output = output;
                return latest_output;
            }
        }

        if (confidence_is_better_by_percent(candidate_anchor_confidence, local_reference_confidence, reference_switch_margin_percent) == false)
        {
            latest_output = output;
            return latest_output;
        }

        if (request_is_allowed_by_interval(candidate_anchor_confidence, now_ms) == false)
        {
            latest_output = output;
            return latest_output;
        }

        if (local_position.pose_id == 0U)
        {
            latest_output = output;
            return latest_output;
        }

        output.request = start_pending_request(local_position, global_position, candidate_anchor_confidence, now_ms);
        output.pending = pending_reference.pending;
        output.pending_pose_id = pending_reference.source_pose_id;
        output.pending_branch_id = pending_reference.source_branch_id;
        output.pending_global_sample_id = pending_reference.global_reference.sample_id;
        if (output.request.has_request == true)
        {
            output.request_reason = request_reason_switch_margin;
        }
        latest_output = output;

        return latest_output;
    }

    output_snapshot read_output()
    {
        return latest_output;
    }
}
