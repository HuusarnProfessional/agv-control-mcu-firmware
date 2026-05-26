#include "anchor_selector.hpp"

#include "anchor_selector_tuning.hpp"
#include "internal/anchor_guards.hpp"
#include "../internal/confidence_math.hpp"
#include "../internal/geometry_helpers.hpp"

#include <cmath>

namespace
{
    struct pending_reference_state
    {
        bool pending = false;
        std::uint32_t request_time_ms = 0U;
        position_sensorfusion_anchors::candidate candidate = {};
        bool has_saved_global_heading = false;
        std::int32_t saved_global_heading_urad = 0;
    };

    pending_reference_state pending_reference = {};
    bool settling = false;
    std::uint32_t settling_start_time_ms = 0U;
    bool has_startup_reference = false;
    std::uint32_t startup_time_ms = 0U;
    std::int64_t startup_local_x_um = 0;
    std::int64_t startup_local_y_um = 0;
    bool has_last_request = false;
    std::uint32_t last_request_time_ms = 0U;
    std::uint32_t last_request_sample_id = 0U;
    std::uint16_t last_request_confidence = 0U;
    bool position_jump_guard_enabled = true;

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
        const std::uint32_t margin = static_cast<std::uint32_t>(reference_confidence) * static_cast<std::uint32_t>(margin_percent) / 100U;
        const std::uint32_t required_confidence = static_cast<std::uint32_t>(reference_confidence) + margin;

        if (required_confidence >= position_sensorfusion_internal::full_confidence)
        {
            return false;
        }

        if (candidate_confidence > required_confidence)
        {
            return true;
        }

        return false;
    }

    void fill_decision_candidates(anchor_selector::decision_snapshot &decision, const position_sensorfusion_anchors::candidate &heading_candidate, const position_sensorfusion_anchors::candidate &position_candidate)
    {
        decision.valid = true;
        decision.heading_confidence = heading_candidate.confidence;
        decision.position_confidence = position_candidate.confidence;
        decision.heading_pose_id = heading_candidate.reference.pose_id;
        decision.position_pose_id = position_candidate.reference.pose_id;
        decision.heading_sample_id = heading_candidate.reference.sample_id;
        decision.position_sample_id = position_candidate.reference.sample_id;
        decision.position_reference_x_um = position_candidate.reference.x_um;
        decision.position_reference_y_um = position_candidate.reference.y_um;
    }

    void fill_position_projection(anchor_selector::decision_snapshot &decision,
                                  const motion_mcu_incoming_state::local_position_state &local_position,
                                  const position_sensorfusion_anchors::current_reference &current_reference,
                                  const position_sensorfusion_anchors::candidate &position_candidate)
    {
        const anchor_guards::projected_candidate_snapshot projection = anchor_guards::build_projected_candidate_snapshot(local_position, current_reference, position_candidate);
        decision.position_projection_valid = projection.valid;
        decision.position_projected_x_um = projection.projected_x_um;
        decision.position_projected_y_um = projection.projected_y_um;
        decision.position_jump_x_um = projection.jump_x_um;
        decision.position_jump_y_um = projection.jump_y_um;
        decision.position_projection_rotation_urad = projection.rotation_urad;
    }

    std::uint16_t calculate_curve_gain_permille(std::int32_t heading_delta_urad, std::uint16_t minimum_gain_permille)
    {
        if (heading_delta_urad <= anchor_selector_tuning::position_forward_curve_start_urad)
        {
            return 1000U;
        }

        if (heading_delta_urad >= anchor_selector_tuning::position_forward_curve_full_urad)
        {
            return minimum_gain_permille;
        }

        const std::int32_t span_urad = anchor_selector_tuning::position_forward_curve_full_urad - anchor_selector_tuning::position_forward_curve_start_urad;
        const std::int32_t progress_urad = heading_delta_urad - anchor_selector_tuning::position_forward_curve_start_urad;
        const std::int32_t gain_span = 1000 - static_cast<std::int32_t>(minimum_gain_permille);
        const std::int32_t reduced_gain = (progress_urad * gain_span) / span_urad;
        const std::int32_t gain_permille = 1000 - reduced_gain;

        if (gain_permille <= static_cast<std::int32_t>(minimum_gain_permille))
        {
            return minimum_gain_permille;
        }

        return static_cast<std::uint16_t>(gain_permille);
    }

    position_sensorfusion_anchors::candidate adjust_position_candidate_for_curve_progress(const motion_mcu_incoming_state::local_position_state &local_position,
                                                                                          const position_sensorfusion_anchors::current_reference &current_reference,
                                                                                          const position_sensorfusion_anchors::candidate &position_candidate)
    {
        position_sensorfusion_anchors::candidate adjusted_candidate = position_candidate;

        if (position_candidate.valid == false)
        {
            return adjusted_candidate;
        }

        if (position_candidate.type != position_sensorfusion_anchors::anchor_type::position_only)
        {
            return adjusted_candidate;
        }

        if (position_candidate.reference.has_local_reference == false)
        {
            return adjusted_candidate;
        }

        if (current_reference.has_heading == false)
        {
            return adjusted_candidate;
        }

        const anchor_guards::projected_candidate_snapshot projection = anchor_guards::build_projected_candidate_snapshot(local_position, current_reference, position_candidate);

        if (projection.valid == false)
        {
            return adjusted_candidate;
        }

        const std::int32_t heading_delta_urad = position_sensorfusion_internal::absolute_angle_delta_urad(local_position.heading_urad, position_candidate.reference.local_heading_urad);
        const std::uint16_t forward_gain_permille = calculate_curve_gain_permille(heading_delta_urad, anchor_selector_tuning::position_forward_min_gain_permille);
        const std::uint16_t lateral_gain_permille = calculate_curve_gain_permille(heading_delta_urad, anchor_selector_tuning::position_lateral_min_gain_permille);

        if ((forward_gain_permille >= 1000U) && (lateral_gain_permille >= 1000U))
        {
            return adjusted_candidate;
        }

        std::int64_t forward_axis_x = 0;
        std::int64_t forward_axis_y = 0;
        position_sensorfusion_internal::rotate_xy_um(1000000, 0, current_reference.heading_urad, forward_axis_x, forward_axis_y);
        const double axis_x = static_cast<double>(forward_axis_x) / 1000000.0;
        const double axis_y = static_cast<double>(forward_axis_y) / 1000000.0;
        const double jump_x = static_cast<double>(projection.jump_x_um);
        const double jump_y = static_cast<double>(projection.jump_y_um);
        const double forward_component_um = (jump_x * axis_x) + (jump_y * axis_y);
        const double forward_delta_x_um = forward_component_um * axis_x;
        const double forward_delta_y_um = forward_component_um * axis_y;
        const double lateral_delta_x_um = jump_x - forward_delta_x_um;
        const double lateral_delta_y_um = jump_y - forward_delta_y_um;
        const double forward_gain = static_cast<double>(forward_gain_permille) / 1000.0;
        const double lateral_gain = static_cast<double>(lateral_gain_permille) / 1000.0;
        const double adjusted_jump_x_um = (lateral_delta_x_um * lateral_gain) + (forward_delta_x_um * forward_gain);
        const double adjusted_jump_y_um = (lateral_delta_y_um * lateral_gain) + (forward_delta_y_um * forward_gain);
        const double projection_adjust_x_um = adjusted_jump_x_um - jump_x;
        const double projection_adjust_y_um = adjusted_jump_y_um - jump_y;

        adjusted_candidate.reference.x_um += static_cast<std::int64_t>(std::llround(projection_adjust_x_um));
        adjusted_candidate.reference.y_um += static_cast<std::int64_t>(std::llround(projection_adjust_y_um));
        return adjusted_candidate;
    }

    bool candidate_has_required_shape(const position_sensorfusion_anchors::candidate &candidate, anchor_selector::reject_reason &reason_out)
    {
        if (candidate.valid == false)
        {
            reason_out = anchor_selector::reject_reason::invalid_candidate;
            return false;
        }

        if (candidate.reference.valid == false)
        {
            reason_out = anchor_selector::reject_reason::invalid_reference;
            return false;
        }

        if (candidate.reference.has_local_reference == false)
        {
            reason_out = anchor_selector::reject_reason::missing_local_reference;
            return false;
        }

        if (candidate.reference.pose_id == 0U)
        {
            reason_out = anchor_selector::reject_reason::missing_pose_id;
            return false;
        }

        if (candidate.confidence < anchor_selector_tuning::minimum_anchor_confidence)
        {
            reason_out = anchor_selector::reject_reason::confidence_low;
            return false;
        }

        if (candidate.type == position_sensorfusion_anchors::anchor_type::heading_transform)
        {
            if (candidate.reference.has_heading == false)
            {
                reason_out = anchor_selector::reject_reason::missing_heading;
                return false;
            }
        }

        reason_out = anchor_selector::reject_reason::none;
        return true;
    }

    bool candidate_is_allowed(const motion_mcu_incoming_state::local_position_state &local_position, const position_sensorfusion_anchors::current_reference &current_reference, const position_sensorfusion_anchors::candidate &candidate, anchor_selector::reject_reason &reason_out)
    {
        if (candidate_has_required_shape(candidate, reason_out) == false)
        {
            return false;
        }

        if (anchor_guards::reference_is_inside_safe_area(candidate) == false)
        {
            reason_out = anchor_selector::reject_reason::outside_safe_area;
            return false;
        }

        if (anchor_guards::candidate_pose_can_be_replayed(local_position, candidate) == false)
        {
            reason_out = anchor_selector::reject_reason::pose_not_replayable;
            return false;
        }

        if (anchor_guards::heading_is_consistent(current_reference, candidate) == false)
        {
            reason_out = anchor_selector::reject_reason::heading_inconsistent;
            return false;
        }

        if ((position_jump_guard_enabled == true) && (anchor_guards::candidate_position_jump_is_safe(local_position, current_reference, candidate) == false))
        {
            reason_out = anchor_selector::reject_reason::jump_too_large;
            return false;
        }

        reason_out = anchor_selector::reject_reason::none;
        return true;
    }

    const position_sensorfusion_anchors::candidate *select_candidate(const motion_mcu_incoming_state::local_position_state &local_position, const position_sensorfusion_anchors::current_reference &current_reference, const position_sensorfusion_anchors::candidate &heading_candidate, const position_sensorfusion_anchors::candidate &position_candidate, std::uint16_t reference_confidence, std::uint16_t margin_percent, anchor_selector::decision_snapshot &decision)
    {
        decision.reason = anchor_selector::reject_reason::none;
        decision.required_confidence = calculate_margin_confidence(reference_confidence, margin_percent);

        anchor_selector::reject_reason reason = anchor_selector::reject_reason::none;

        if (candidate_is_allowed(local_position, current_reference, heading_candidate, reason) == true)
        {
            if (confidence_beats_margin(heading_candidate.confidence, reference_confidence, margin_percent) == true)
            {
                decision.selected_type = heading_candidate.type;
                return &heading_candidate;
            }

            decision.reason = anchor_selector::reject_reason::margin_not_met;
        }
        else if (decision.reason == anchor_selector::reject_reason::none)
        {
            decision.reason = reason;
        }

        if (current_reference.valid == false)
        {
            return nullptr;
        }

        if (candidate_is_allowed(local_position, current_reference, position_candidate, reason) == true)
        {
            if (confidence_beats_margin(position_candidate.confidence, reference_confidence, margin_percent) == true)
            {
                decision.selected_type = position_candidate.type;
                return &position_candidate;
            }

            decision.reason = anchor_selector::reject_reason::margin_not_met;
        }
        else if (decision.reason == anchor_selector::reject_reason::none)
        {
            decision.reason = reason;
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

    void update_startup_reference(const motion_mcu_incoming_state::local_position_state &local_position, std::uint32_t now_ms)
    {
        if (has_startup_reference == true)
        {
            return;
        }

        if (local_position.has_pose == false)
        {
            return;
        }

        has_startup_reference = true;
        startup_time_ms = now_ms;
        startup_local_x_um = local_position.x_um;
        startup_local_y_um = local_position.y_um;
    }

    bool startup_delay_has_passed(const motion_mcu_incoming_state::local_position_state &local_position, std::uint32_t now_ms)
    {
        if (has_startup_reference == false)
        {
            return false;
        }

        const std::uint32_t startup_age_ms = position_sensorfusion_internal::elapsed_ms(now_ms, startup_time_ms);

        if (startup_age_ms < anchor_selector_tuning::startup_anchor_delay_ms)
        {
            return false;
        }

        const std::int64_t delta_x_um = local_position.x_um - startup_local_x_um;
        const std::int64_t delta_y_um = local_position.y_um - startup_local_y_um;
        const std::int64_t travel_um = position_sensorfusion_internal::calculate_distance_um(delta_x_um, delta_y_um);

        if (travel_um < anchor_selector_tuning::startup_minimum_travel_um)
        {
            return false;
        }

        return true;
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
        activation.has_saved_global_heading = pending_reference.has_saved_global_heading;
        activation.source_pose_id = pending_reference.candidate.reference.pose_id;
        activation.source_branch_id = pending_reference.candidate.reference.branch_id;
        activation.activation_time_ms = now_ms;
        activation.confidence = pending_reference.candidate.confidence;
        activation.saved_global_heading_urad = pending_reference.saved_global_heading_urad;
        activation.reference = pending_reference.candidate.reference;
        pending_reference = {};
        settling = true;
        settling_start_time_ms = now_ms;

        return activation;
    }

    position_sensorfusion_anchors::branch_request start_request(const position_sensorfusion_anchors::current_reference &current_reference,
                                                                const position_sensorfusion_anchors::candidate &candidate,
                                                                std::uint32_t now_ms,
                                                                anchor_selector::decision_snapshot &decision)
    {
        position_sensorfusion_anchors::branch_request request = {};

        if (candidate.reference.sample_id == last_request_sample_id)
        {
            decision.reason = anchor_selector::reject_reason::duplicate_sample;
            return request;
        }

        if (request_interval_has_passed(candidate.confidence, now_ms) == false)
        {
            decision.reason = anchor_selector::reject_reason::request_interval;
            return request;
        }

        pending_reference.pending = true;
        pending_reference.request_time_ms = now_ms;
        pending_reference.candidate = candidate;
        pending_reference.has_saved_global_heading = current_reference.has_heading;
        pending_reference.saved_global_heading_urad = current_reference.heading_urad;
        has_last_request = true;
        last_request_time_ms = now_ms;
        last_request_sample_id = candidate.reference.sample_id;
        last_request_confidence = candidate.confidence;

        request.valid = true;
        request.type = candidate.type;
        request.pose_id = candidate.reference.pose_id;
        request.branch_id = candidate.reference.branch_id;
        request.confidence = candidate.confidence;
        decision.state = anchor_selector::decision_state::requested;
        decision.reason = anchor_selector::reject_reason::none;

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
        has_startup_reference = false;
        startup_time_ms = 0U;
        startup_local_x_um = 0;
        startup_local_y_um = 0;
        has_last_request = false;
        last_request_time_ms = 0U;
        last_request_sample_id = 0U;
        last_request_confidence = 0U;
    }

    void set_position_jump_guard_enabled(bool enabled)
    {
        position_jump_guard_enabled = enabled;
    }

    bool is_position_jump_guard_enabled()
    {
        return position_jump_guard_enabled;
    }

    output_snapshot update(const motion_mcu_incoming_state::local_position_state &local_position, const position_sensorfusion_anchors::current_reference &current_reference, const position_sensorfusion_anchors::candidate &heading_candidate, const position_sensorfusion_anchors::candidate &position_candidate, std::uint32_t now_ms)
    {
        update_settling(now_ms);
        update_pending_timeout(now_ms);

        const position_sensorfusion_anchors::candidate adjusted_position_candidate = adjust_position_candidate_for_curve_progress(local_position, current_reference, position_candidate);
        output_snapshot output = build_output();
        fill_decision_candidates(output.decision, heading_candidate, adjusted_position_candidate);
        output.decision.local_confidence = local_position.confidence_position;

        if (local_position.has_pose == false)
        {
            output.decision.state = decision_state::local_missing;
            return output;
        }

        update_startup_reference(local_position, now_ms);
        fill_position_projection(output.decision, local_position, current_reference, adjusted_position_candidate);

        if (pending_branch_has_arrived(local_position) == true)
        {
            output.activation = activate_pending_reference(now_ms);
            output.pending = false;
            output.settling = true;
            output.decision.state = decision_state::activated;
            return output;
        }

        if (current_reference.valid == false)
        {
            output.decision.state = decision_state::missing_current_reference;
            return output;
        }

        if (pending_reference.pending == true)
        {
            output.decision.state = decision_state::pending;
            return output;
        }

        if (settling == true)
        {
            output.decision.state = decision_state::settling;
            return output;
        }

        if (startup_delay_has_passed(local_position, now_ms) == false)
        {
            output.decision.state = decision_state::rejected;
            output.decision.reason = reject_reason::startup_delay;
            return output;
        }

        const std::uint16_t local_reference_confidence = local_position.confidence_position;
        const position_sensorfusion_anchors::candidate *candidate = select_candidate(local_position, current_reference, heading_candidate, adjusted_position_candidate, local_reference_confidence, anchor_selector_tuning::reference_switch_margin_percent, output.decision);

        if (candidate == nullptr)
        {
            output.decision.state = decision_state::rejected;
            return output;
        }

        output.request = start_request(current_reference, *candidate, now_ms, output.decision);
        output.pending = pending_reference.pending;

        if (output.request.valid == false)
        {
            output.decision.state = decision_state::rejected;
        }

        return output;
    }
}
