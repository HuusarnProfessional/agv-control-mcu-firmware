#include "anchor_selector.hpp"

#include "anchor_selector_tuning.hpp"
#include "internal/anchor_guards.hpp"
#include "../internal/confidence_math.hpp"
#include "../internal/geometry_helpers.hpp"

namespace
{
    struct pending_reference_state
    {
        bool pending = false;
        std::uint32_t request_time_ms = 0U;
        position_sensorfusion_anchors::candidate candidate = {};
    };

    pending_reference_state pending_reference = {};
    bool settling = false;
    std::uint32_t settling_start_time_ms = 0U;
    bool has_last_request = false;
    std::uint32_t last_request_time_ms = 0U;
    std::uint32_t last_request_sample_id = 0U;
    std::uint16_t last_request_confidence = 0U;

    std::uint16_t calculate_margin_confidence(std::uint16_t confidence, std::uint16_t margin_percent)
    {
        const std::uint32_t margin = static_cast<std::uint32_t>(confidence) * static_cast<std::uint32_t>(margin_percent) / 100U;
        const std::uint32_t required_confidence = static_cast<std::uint32_t>(confidence) + margin;

        if (required_confidence > position_sensorfusion_internal::full_confidence)
        {
            return position_sensorfusion_internal::full_confidence;
        }

        return static_cast<std::uint16_t>(required_confidence);
    }

    bool confidence_beats_margin(std::uint16_t candidate_confidence, std::uint16_t reference_confidence, std::uint16_t margin_percent)
    {
        const std::uint16_t required_confidence = calculate_margin_confidence(reference_confidence, margin_percent);

        if ((required_confidence == position_sensorfusion_internal::full_confidence) && (candidate_confidence == position_sensorfusion_internal::full_confidence))
        {
            return true;
        }

        if (candidate_confidence > required_confidence)
        {
            return true;
        }

        return false;
    }

    bool candidate_has_required_shape(const position_sensorfusion_anchors::candidate &candidate)
    {
        if (candidate.valid == false)
        {
            return false;
        }

        if (candidate.reference.valid == false)
        {
            return false;
        }

        if (candidate.reference.has_local_reference == false)
        {
            return false;
        }

        if (candidate.reference.pose_id == 0U)
        {
            return false;
        }

        if (candidate.confidence < anchor_selector_tuning::minimum_anchor_confidence)
        {
            return false;
        }

        if (candidate.type == position_sensorfusion_anchors::anchor_type::heading_transform)
        {
            if (candidate.reference.has_heading == false)
            {
                return false;
            }
        }

        return true;
    }

    bool candidate_is_allowed(const motion_mcu_incoming_state::local_position_state &local_position, const position_sensorfusion_anchors::current_reference &current_reference, const position_sensorfusion_anchors::candidate &candidate)
    {
        if (candidate_has_required_shape(candidate) == false)
        {
            return false;
        }

        if (anchor_guards::reference_is_inside_safe_area(candidate) == false)
        {
            return false;
        }

        if (anchor_guards::candidate_pose_can_be_replayed(local_position, candidate) == false)
        {
            return false;
        }

        if (anchor_guards::heading_is_consistent(current_reference, candidate) == false)
        {
            return false;
        }

        if (anchor_guards::candidate_position_jump_is_safe(local_position, current_reference, candidate) == false)
        {
            return false;
        }

        return true;
    }

    const position_sensorfusion_anchors::candidate *select_candidate(const motion_mcu_incoming_state::local_position_state &local_position, const position_sensorfusion_anchors::current_reference &current_reference, const position_sensorfusion_anchors::candidate &heading_candidate, const position_sensorfusion_anchors::candidate &position_candidate, std::uint16_t reference_confidence, std::uint16_t margin_percent)
    {
        if (candidate_is_allowed(local_position, current_reference, heading_candidate) == true)
        {
            if (confidence_beats_margin(heading_candidate.confidence, reference_confidence, margin_percent) == true)
            {
                return &heading_candidate;
            }
        }

        if (current_reference.valid == false)
        {
            return nullptr;
        }

        if (candidate_is_allowed(local_position, current_reference, position_candidate) == true)
        {
            if (confidence_beats_margin(position_candidate.confidence, reference_confidence, margin_percent) == true)
            {
                return &position_candidate;
            }
        }

        return nullptr;
    }

    bool request_interval_has_passed(std::uint16_t confidence, std::uint32_t now_ms)
    {
        if (has_last_request == false)
        {
            return true;
        }

        const std::uint32_t request_age_ms = position_sensorfusion_internal::elapsed_ms(now_ms, last_request_time_ms);

        if (request_age_ms >= anchor_selector_tuning::minimum_request_interval_ms)
        {
            return true;
        }

        return confidence_beats_margin(confidence, last_request_confidence, anchor_selector_tuning::pending_switch_margin_percent);
    }

    void update_pending_timeout(std::uint32_t now_ms)
    {
        if (pending_reference.pending == false)
        {
            return;
        }

        const std::uint32_t pending_age_ms = position_sensorfusion_internal::elapsed_ms(now_ms, pending_reference.request_time_ms);

        if (pending_age_ms >= anchor_selector_tuning::pending_request_timeout_ms)
        {
            pending_reference = {};
        }
    }

    void update_settling(std::uint32_t now_ms)
    {
        if (settling == false)
        {
            return;
        }

        const std::uint32_t settling_age_ms = position_sensorfusion_internal::elapsed_ms(now_ms, settling_start_time_ms);

        if (settling_age_ms >= anchor_selector_tuning::settling_time_ms)
        {
            settling = false;
        }
    }

    bool pending_branch_has_arrived(const motion_mcu_incoming_state::local_position_state &local_position)
    {
        if (pending_reference.pending == false)
        {
            return false;
        }

        if (local_position.branch_id != pending_reference.candidate.reference.branch_id)
        {
            return true;
        }

        return false;
    }

    position_sensorfusion_anchors::reference_activation activate_pending_reference(std::uint32_t now_ms)
    {
        position_sensorfusion_anchors::reference_activation activation = {};
        activation.valid = true;
        activation.type = pending_reference.candidate.type;
        activation.source_pose_id = pending_reference.candidate.reference.pose_id;
        activation.source_branch_id = pending_reference.candidate.reference.branch_id;
        activation.activation_time_ms = now_ms;
        activation.confidence = pending_reference.candidate.confidence;
        activation.reference = pending_reference.candidate.reference;
        pending_reference = {};
        settling = true;
        settling_start_time_ms = now_ms;

        return activation;
    }

    position_sensorfusion_anchors::branch_request start_request(const position_sensorfusion_anchors::candidate &candidate, std::uint32_t now_ms)
    {
        position_sensorfusion_anchors::branch_request request = {};

        if (candidate.reference.sample_id == last_request_sample_id)
        {
            return request;
        }

        if (request_interval_has_passed(candidate.confidence, now_ms) == false)
        {
            return request;
        }

        pending_reference.pending = true;
        pending_reference.request_time_ms = now_ms;
        pending_reference.candidate = candidate;
        has_last_request = true;
        last_request_time_ms = now_ms;
        last_request_sample_id = candidate.reference.sample_id;
        last_request_confidence = candidate.confidence;

        request.valid = true;
        request.type = candidate.type;
        request.pose_id = candidate.reference.pose_id;
        request.branch_id = candidate.reference.branch_id;
        request.confidence = candidate.confidence;

        return request;
    }

    anchor_selector::output_snapshot build_output()
    {
        anchor_selector::output_snapshot output = {};
        output.pending = pending_reference.pending;
        output.settling = settling;

        return output;
    }
}

namespace anchor_selector
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
        last_request_sample_id = 0U;
        last_request_confidence = 0U;
    }

    output_snapshot update(const motion_mcu_incoming_state::local_position_state &local_position, const position_sensorfusion_anchors::current_reference &current_reference, const position_sensorfusion_anchors::candidate &heading_candidate, const position_sensorfusion_anchors::candidate &position_candidate, std::uint32_t now_ms)
    {
        update_settling(now_ms);
        update_pending_timeout(now_ms);

        output_snapshot output = build_output();

        if (local_position.has_pose == false)
        {
            return output;
        }

        if (pending_branch_has_arrived(local_position) == true)
        {
            output.activation = activate_pending_reference(now_ms);
            output.pending = false;
            output.settling = true;
            return output;
        }

        if (current_reference.valid == false)
        {
            return output;
        }

        if (pending_reference.pending == true)
        {
            return output;
        }

        if (settling == true)
        {
            return output;
        }

        const std::uint16_t local_reference_confidence = local_position.confidence_position;
        const position_sensorfusion_anchors::candidate *candidate = select_candidate(local_position, current_reference, heading_candidate, position_candidate, local_reference_confidence, anchor_selector_tuning::reference_switch_margin_percent);

        if (candidate == nullptr)
        {
            return output;
        }

        output.request = start_request(*candidate, now_ms);
        output.pending = pending_reference.pending;
        return output;
    }
}
