#include "global_reference_selector.hpp"

#include <cstdint>

namespace
{
    constexpr std::uint16_t full_confidence = 1000U;
    constexpr std::uint16_t minimum_reference_score = 300U;
    constexpr std::uint16_t reference_switch_margin_percent = 20U;
    constexpr std::uint16_t interval_override_margin_percent = 50U;
    constexpr std::uint32_t minimum_request_interval_ms = 1000U;
    constexpr std::uint32_t pending_request_timeout_ms = 3000U;
    constexpr std::uint32_t settling_time_ms = 500U;

    struct pending_reference_state
    {
        bool pending = false;
        std::uint8_t source_pose_id = 0U;
        std::uint8_t source_branch_id = 0U;
        std::uint32_t request_time_ms = 0U;
        std::uint16_t reference_score = 0U;
        filtered_global_position::output_snapshot global_reference = {};
    };

    pending_reference_state pending_reference = {};
    bool settling = false;
    std::uint32_t settling_start_time_ms = 0U;
    bool has_last_request = false;
    std::uint32_t last_request_time_ms = 0U;
    std::uint32_t last_request_global_sample_id = 0U;
    std::uint16_t last_request_score = 0U;
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

    std::uint16_t calculate_margin_score(std::uint16_t score, std::uint16_t margin_percent)
    {
        const std::uint32_t margin = static_cast<std::uint32_t>(score) * static_cast<std::uint32_t>(margin_percent) / 100U;
        const std::uint32_t required_score = static_cast<std::uint32_t>(score) + margin;

        if (required_score > full_confidence)
        {
            return full_confidence;
        }

        return static_cast<std::uint16_t>(required_score);
    }

    bool score_is_better_by_percent(std::uint16_t new_score, std::uint16_t old_score, std::uint16_t margin_percent)
    {
        const std::uint16_t required_score = calculate_margin_score(old_score, margin_percent);

        if (new_score > required_score)
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

        return true;
    }

    std::uint16_t calculate_global_reference_score(const filtered_global_position::output_snapshot &global_position)
    {
        if (global_position_can_be_reference(global_position) == false)
        {
            return 0U;
        }

        return smaller_confidence(global_position.confidence_position, global_position.confidence_heading);
    }

    std::uint16_t calculate_current_reference_score(const global_reference_selector::current_reference_snapshot &current_reference)
    {
        if (current_reference.has_reference == false)
        {
            return 0U;
        }

        return smaller_confidence(current_reference.confidence_position, current_reference.confidence_heading);
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

    void mark_last_request(const filtered_global_position::output_snapshot &global_position, std::uint16_t reference_score, std::uint32_t now_ms)
    {
        has_last_request = true;
        last_request_time_ms = now_ms;
        last_request_global_sample_id = global_position.sample_id;
        last_request_score = reference_score;
    }

    global_reference_selector::reference_activation build_initial_activation(const motion_mcu_incoming_state::local_position_state &local_position, const filtered_global_position::output_snapshot &global_position, std::uint16_t reference_score, std::uint32_t now_ms)
    {
        global_reference_selector::reference_activation activation = {};

        activation.has_activation = true;
        activation.is_initial_reference = true;
        activation.is_mission_seed = false;
        activation.source_pose_id = local_position.pose_id;
        activation.source_branch_id = local_position.branch_id;
        activation.activation_time_ms = now_ms;
        activation.reference_score = reference_score;
        activation.global_reference = global_position;

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
        activation.reference_score = pending_reference.reference_score;
        activation.global_reference = pending_reference.global_reference;

        pending_reference = {};
        settling = true;
        settling_start_time_ms = now_ms;

        return activation;
    }

    global_reference_selector::branch_request start_pending_request(const motion_mcu_incoming_state::local_position_state &local_position, const filtered_global_position::output_snapshot &global_position, std::uint16_t reference_score, std::uint32_t now_ms)
    {
        global_reference_selector::branch_request request = {};

        pending_reference.pending = true;
        pending_reference.source_pose_id = local_position.pose_id;
        pending_reference.source_branch_id = local_position.branch_id;
        pending_reference.request_time_ms = now_ms;
        pending_reference.reference_score = reference_score;
        pending_reference.global_reference = global_position;

        mark_last_request(global_position, reference_score, now_ms);

        request.has_request = true;
        request.pose_id = local_position.pose_id;
        request.branch_id = local_position.branch_id;
        request.reference_score = reference_score;

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

    bool request_is_allowed_by_interval(std::uint16_t reference_score, std::uint32_t now_ms)
    {
        if (normal_request_interval_has_passed(now_ms) == true)
        {
            return true;
        }

        if (score_is_better_by_percent(reference_score, last_request_score, interval_override_margin_percent) == true)
        {
            return true;
        }

        return false;
    }

    global_reference_selector::output_snapshot build_output(std::uint16_t global_reference_score, std::uint16_t current_reference_score)
    {
        global_reference_selector::output_snapshot output = {};

        output.pending = pending_reference.pending;
        output.settling = settling;
        output.pending_pose_id = pending_reference.source_pose_id;
        output.pending_branch_id = pending_reference.source_branch_id;
        output.pending_global_sample_id = pending_reference.global_reference.sample_id;
        output.global_reference_score = global_reference_score;
        output.current_reference_score = current_reference_score;

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
        last_request_score = 0U;
        latest_output = {};
    }

    output_snapshot update(const motion_mcu_incoming_state::local_position_state &local_position, const filtered_global_position::output_snapshot &global_position, const current_reference_snapshot &current_reference, std::uint32_t now_ms)
    {
        update_settling(now_ms);
        update_pending_timeout(now_ms);

        const std::uint16_t global_reference_score = calculate_global_reference_score(global_position);
        const std::uint16_t current_reference_score = calculate_current_reference_score(current_reference);
        output_snapshot output = build_output(global_reference_score, current_reference_score);

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

        if (global_reference_score < minimum_reference_score)
        {
            latest_output = output;
            return latest_output;
        }

        if (current_reference.has_reference == false)
        {
            output.activation = build_initial_activation(local_position, global_position, global_reference_score, now_ms);
            latest_output = output;
            return latest_output;
        }

        if (pending_reference.pending == true)
        {
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

        if (score_is_better_by_percent(global_reference_score, current_reference_score, reference_switch_margin_percent) == false)
        {
            latest_output = output;
            return latest_output;
        }

        if (request_is_allowed_by_interval(global_reference_score, now_ms) == false)
        {
            latest_output = output;
            return latest_output;
        }

        if (local_position.pose_id == 0U)
        {
            latest_output = output;
            return latest_output;
        }

        output.request = start_pending_request(local_position, global_position, global_reference_score, now_ms);
        output.pending = pending_reference.pending;
        output.pending_pose_id = pending_reference.source_pose_id;
        output.pending_branch_id = pending_reference.source_branch_id;
        output.pending_global_sample_id = pending_reference.global_reference.sample_id;
        latest_output = output;

        return latest_output;
    }

    output_snapshot read_output()
    {
        return latest_output;
    }
}
