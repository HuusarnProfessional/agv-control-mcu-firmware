#include "position_sensorfusion_pipeline.hpp"

#include "anchors/anchor_selector.hpp"
#include "anchors/heading_anchor/heading_anchor.hpp"
#include "anchors/position_anchor/position_anchor.hpp"
#include "filtered_global/filtered_global_pipeline.hpp"
#include "filtered_global/filtered_global_tuning.hpp"
#include "seed/mission_reference_seed.hpp"
#include "transform/local_to_global_transform.hpp"
#include "position_sensorfusion.hpp"
#include "internal/confidence_math.hpp"
#include "internal/geometry_helpers.hpp"
#include "../mission/mission_runner.hpp"
#include "../motion_mcu_communication/state/incoming/incoming_state.hpp"
#include "../motion_mcu_communication/outgoing_payloads/service/position_correction_payload.hpp"
#include "../position_trace/position_trace_logger.hpp"
#include "../control/primitives/motion_primitive/motion_primitive_status_monitor.hpp"
#include "../pure_pursuit/pure_pursuit.hpp"
#include "../pure_pursuit/pure_pursuit_pipeline.hpp"

#include <cmath>

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
    bool stop_relocalize_test_enabled = false;

    constexpr std::uint16_t stop_relocalize_stop_confidence = 650U;
    constexpr std::uint16_t stop_relocalize_resume_confidence = 850U;
    constexpr std::uint32_t stop_relocalize_stationary_settle_time_ms = 500U;
    constexpr std::uint32_t stop_relocalize_stationary_collect_time_ms = 3000U;
    constexpr std::uint32_t stop_relocalize_max_filtered_age_ms = 1500U;
    constexpr std::uint32_t stop_relocalize_cooldown_ms = 5000U;
    constexpr std::uint32_t stop_relocalize_branch_request_timeout_ms = 3000U;
    constexpr std::uint32_t stop_relocalize_max_hold_time_ms = 10000U;
    constexpr std::uint32_t stop_relocalize_stationary_reset_distance_um = 50000U;
    constexpr std::uint8_t stop_relocalize_minimum_sample_count = 4U;
    constexpr std::uint8_t stop_relocalize_full_sample_count = 10U;
    constexpr std::uint32_t stop_relocalize_good_spread_um = 100000U;
    constexpr std::uint32_t stop_relocalize_zero_spread_um = 600000U;

    enum class stop_relocalize_phase : std::uint8_t
    {
        disabled = 0U,
        monitoring = 1U,
        holding_settle = 2U,
        holding_collect = 3U,
        request_wait_branch = 4U,
        cooldown = 5U,
        blocked = 6U
    };

    enum class stop_relocalize_cancel_reason : std::uint8_t
    {
        none = 0U,
        candidate_failed = 1U,
        branch_timeout = 2U,
        max_hold_timeout = 3U
    };

    struct stop_relocalize_pending_request
    {
        bool pending = false;
        std::uint32_t request_time_ms = 0U;
        position_sensorfusion_anchors::candidate candidate = {};
        bool has_saved_global_heading = false;
        std::int32_t saved_global_heading_urad = 0;
    };

    struct stop_relocalize_runtime
    {
        bool active = false;
        std::uint32_t attempt_start_time_ms = 0U;
        std::uint32_t phase_start_time_ms = 0U;
        std::uint32_t phase_required_time_ms = 0U;
        std::uint32_t cooldown_until_ms = 0U;
        std::int64_t stationary_reference_x_um = 0;
        std::int64_t stationary_reference_y_um = 0;
        stop_relocalize_pending_request pending_request = {};
        bool last_activation_performed = false;
        std::uint16_t last_activation_confidence = 0U;
        std::uint32_t last_activation_time_ms = 0U;
        stop_relocalize_cancel_reason last_cancel_reason = stop_relocalize_cancel_reason::none;
        std::uint16_t last_local_confidence_position = 0U;
        std::uint16_t last_filtered_confidence_position = 0U;
        bool last_filtered_ready = false;
        std::uint32_t last_filtered_age_ms = 0U;
        std::uint8_t last_collected_sample_count = 0U;
        stop_relocalize_phase phase = stop_relocalize_phase::disabled;
    };

    stop_relocalize_runtime stop_relocalize = {};

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

    void set_stop_relocalize_phase(stop_relocalize_phase phase, std::uint32_t now_ms, std::uint32_t required_time_ms)
    {
        if (stop_relocalize.phase != phase)
        {
            stop_relocalize.phase_start_time_ms = now_ms;
        }

        stop_relocalize.phase = phase;
        stop_relocalize.phase_required_time_ms = required_time_ms;
    }

    void begin_stop_relocalize_stationary_phase(stop_relocalize_phase phase,
                                                std::uint32_t now_ms,
                                                std::uint32_t required_time_ms,
                                                const motion_mcu_incoming_state::local_position_state &local_position)
    {
        if (phase == stop_relocalize_phase::holding_settle)
        {
            stop_relocalize.last_collected_sample_count = 0U;
        }

        stop_relocalize.stationary_reference_x_um = local_position.x_um;
        stop_relocalize.stationary_reference_y_um = local_position.y_um;
        set_stop_relocalize_phase(phase, now_ms, required_time_ms);
    }

    std::uint32_t stop_relocalize_stationary_distance_um(const motion_mcu_incoming_state::local_position_state &local_position)
    {
        const std::int64_t delta_x_um = local_position.x_um - stop_relocalize.stationary_reference_x_um;
        const std::int64_t delta_y_um = local_position.y_um - stop_relocalize.stationary_reference_y_um;
        return static_cast<std::uint32_t>(position_sensorfusion_internal::calculate_distance_um(delta_x_um, delta_y_um));
    }

    bool stop_relocalize_has_stood_still(const motion_mcu_incoming_state::local_position_state &local_position)
    {
        return stop_relocalize_stationary_distance_um(local_position) <= stop_relocalize_stationary_reset_distance_um;
    }

    void reset_stop_relocalize_runtime(bool clear_cooldown)
    {
        stop_relocalize.active = false;
        stop_relocalize.attempt_start_time_ms = 0U;
        stop_relocalize.phase_start_time_ms = 0U;
        stop_relocalize.phase_required_time_ms = 0U;
        stop_relocalize.stationary_reference_x_um = 0;
        stop_relocalize.stationary_reference_y_um = 0;
        stop_relocalize.pending_request = {};
        stop_relocalize.last_filtered_ready = false;
        stop_relocalize.last_filtered_age_ms = 0U;
        stop_relocalize.last_local_confidence_position = 0U;
        stop_relocalize.last_filtered_confidence_position = 0U;
        stop_relocalize.last_collected_sample_count = 0U;
        stop_relocalize.last_activation_performed = false;
        stop_relocalize.last_activation_confidence = 0U;
        stop_relocalize.last_activation_time_ms = 0U;

        if (clear_cooldown == true)
        {
            stop_relocalize.cooldown_until_ms = 0U;
            stop_relocalize.last_cancel_reason = stop_relocalize_cancel_reason::none;
        }

        pure_pursuit::set_external_hold(false);
    }

    bool stop_relocalize_mode_should_run()
    {
        return (stop_relocalize_test_enabled == true) &&
               (mission_runner::is_running() == true) &&
               (local_only_mode == false);
    }

    bool stop_relocalize_attempt_timed_out(std::uint32_t now_ms)
    {
        return (stop_relocalize.attempt_start_time_ms != 0U) &&
               (position_sensorfusion_internal::elapsed_ms(now_ms, stop_relocalize.attempt_start_time_ms) >= stop_relocalize_max_hold_time_ms);
    }

    bool stop_relocalize_branch_request_timed_out(std::uint32_t now_ms)
    {
        return (stop_relocalize.pending_request.pending == true) &&
               (position_sensorfusion_internal::elapsed_ms(now_ms, stop_relocalize.pending_request.request_time_ms) >= stop_relocalize_branch_request_timeout_ms);
    }

    void finish_stop_relocalize_without_activation(std::uint32_t now_ms, stop_relocalize_cancel_reason reason)
    {
        stop_relocalize.active = false;
        stop_relocalize.attempt_start_time_ms = 0U;
        stop_relocalize.pending_request = {};
        stop_relocalize.last_cancel_reason = reason;
        stop_relocalize.cooldown_until_ms = now_ms + stop_relocalize_cooldown_ms;
        set_stop_relocalize_phase(stop_relocalize_phase::cooldown, now_ms, 0U);
        pure_pursuit::set_external_hold(false);
    }

    bool filtered_position_ready_for_stop_relocalize(const filtered_global::output_snapshot &filtered_position, std::uint32_t now_ms)
    {
        if (filtered_position.has_position == false)
        {
            return false;
        }

        if (filtered_position.confidence_position < stop_relocalize_resume_confidence)
        {
            return false;
        }

        if (filtered_position.received_time_ms == 0U)
        {
            return false;
        }

        return position_sensorfusion_internal::elapsed_ms(now_ms, filtered_position.received_time_ms) <= stop_relocalize_max_filtered_age_ms;
    }

    std::uint32_t remaining_ms(std::uint32_t now_ms, std::uint32_t target_ms)
    {
        if (target_ms <= now_ms)
        {
            return 0U;
        }

        return target_ms - now_ms;
    }

    std::uint16_t stop_relocalize_average_sample_confidence(const filtered_global::sample *samples, std::uint8_t sample_count)
    {
        if (sample_count == 0U)
        {
            return 0U;
        }

        std::uint32_t confidence_sum = 0U;

        for (std::uint8_t index = 0U; index < sample_count; index++)
        {
            confidence_sum += samples[index].confidence_position;
        }

        return static_cast<std::uint16_t>(confidence_sum / sample_count);
    }

    bool collect_stop_relocalize_samples(std::uint32_t collection_start_ms,
                                         std::uint8_t current_branch_id,
                                         filtered_global::sample *samples_out,
                                         std::uint8_t &sample_count_out)
    {
        sample_count_out = 0U;

        for (std::uint8_t index = 0U; index < filtered_global::history_count(); index++)
        {
            filtered_global::sample sample = {};

            if (filtered_global::read_history_sample(index, sample) == false)
            {
                continue;
            }

            if ((sample.valid == false) || (sample.has_local_reference == false))
            {
                continue;
            }

            if (sample.branch_id != current_branch_id)
            {
                continue;
            }

            if (sample.received_time_ms < collection_start_ms)
            {
                continue;
            }

            samples_out[sample_count_out] = sample;
            sample_count_out++;

            if (sample_count_out >= filtered_global_tuning::history_size)
            {
                break;
            }
        }

        return sample_count_out > 0U;
    }

    position_sensorfusion_anchors::candidate build_stop_relocalize_candidate(std::uint32_t collection_start_ms,
                                                                             std::uint8_t current_branch_id)
    {
        position_sensorfusion_anchors::candidate candidate = {};
        filtered_global::sample samples[filtered_global_tuning::history_size] = {};
        std::uint8_t sample_count = 0U;

        if (collect_stop_relocalize_samples(collection_start_ms, current_branch_id, samples, sample_count) == false)
        {
            return candidate;
        }

        stop_relocalize.last_collected_sample_count = sample_count;

        if (sample_count < stop_relocalize_minimum_sample_count)
        {
            return candidate;
        }

        const filtered_global::sample &reference_sample = samples[0U];

        if ((reference_sample.pose_id == 0U) || (reference_sample.has_local_reference == false))
        {
            return candidate;
        }

        std::int64_t weighted_x_sum = 0LL;
        std::int64_t weighted_y_sum = 0LL;
        std::int64_t weighted_z_sum = 0LL;
        std::uint32_t weight_sum = 0U;

        for (std::uint8_t index = 0U; index < sample_count; index++)
        {
            std::uint32_t weight = samples[index].confidence_position;

            if (weight == 0U)
            {
                weight = 1U;
            }

            weighted_x_sum += samples[index].x_um * static_cast<std::int64_t>(weight);
            weighted_y_sum += samples[index].y_um * static_cast<std::int64_t>(weight);
            weighted_z_sum += samples[index].z_um * static_cast<std::int64_t>(weight);
            weight_sum += weight;
        }

        if (weight_sum == 0U)
        {
            return candidate;
        }

        const std::int64_t center_x_um = static_cast<std::int64_t>(weighted_x_sum / weight_sum);
        const std::int64_t center_y_um = static_cast<std::int64_t>(weighted_y_sum / weight_sum);
        const std::int64_t center_z_um = static_cast<std::int64_t>(weighted_z_sum / weight_sum);

        std::uint32_t maximum_residual_um = 0U;

        for (std::uint8_t index = 0U; index < sample_count; index++)
        {
            const std::int64_t delta_x_um = samples[index].x_um - center_x_um;
            const std::int64_t delta_y_um = samples[index].y_um - center_y_um;
            const std::uint32_t residual_um = static_cast<std::uint32_t>(position_sensorfusion_internal::calculate_distance_um(delta_x_um, delta_y_um));

            if (residual_um > maximum_residual_um)
            {
                maximum_residual_um = residual_um;
            }
        }

        const std::uint16_t sample_count_confidence =
            position_sensorfusion_internal::sample_count_to_confidence(
                sample_count,
                stop_relocalize_minimum_sample_count,
                stop_relocalize_full_sample_count);
        const std::uint16_t average_confidence = stop_relocalize_average_sample_confidence(samples, sample_count);
        const std::uint16_t spread_confidence =
            position_sensorfusion_internal::range_to_confidence(
                maximum_residual_um,
                stop_relocalize_good_spread_um,
                stop_relocalize_zero_spread_um);

        std::uint16_t confidence = position_sensorfusion_internal::smaller_confidence(sample_count_confidence, average_confidence);
        confidence = position_sensorfusion_internal::smaller_confidence(confidence, spread_confidence);

        if (confidence < stop_relocalize_resume_confidence)
        {
            return candidate;
        }

        candidate.valid = true;
        candidate.type = position_sensorfusion_anchors::anchor_type::position_only;
        candidate.confidence = confidence;
        candidate.reference.valid = true;
        candidate.reference.x_um = center_x_um;
        candidate.reference.y_um = center_y_um;
        candidate.reference.z_um = center_z_um;
        candidate.reference.confidence_position = confidence;
        candidate.reference.sample_id = reference_sample.sample_id;
        candidate.reference.received_time_ms = reference_sample.received_time_ms;
        candidate.reference.has_local_reference = true;
        candidate.reference.local_x_um = reference_sample.local_x_um;
        candidate.reference.local_y_um = reference_sample.local_y_um;
        candidate.reference.local_heading_urad = reference_sample.local_heading_urad;
        candidate.reference.pose_id = reference_sample.pose_id;
        candidate.reference.branch_id = reference_sample.branch_id;

        return candidate;
    }

    bool start_stop_relocalize_request(const position_sensorfusion_anchors::current_reference &current_reference,
                                       const position_sensorfusion_anchors::candidate &candidate,
                                       std::uint32_t now_ms,
                                       position_sensorfusion_anchors::branch_request &request_out)
    {
        if ((candidate.valid == false) || (candidate.reference.valid == false))
        {
            return false;
        }

        stop_relocalize.pending_request.pending = true;
        stop_relocalize.pending_request.request_time_ms = now_ms;
        stop_relocalize.pending_request.candidate = candidate;
        stop_relocalize.pending_request.has_saved_global_heading = current_reference.has_heading;
        stop_relocalize.pending_request.saved_global_heading_urad = current_reference.heading_urad;

        request_out.valid = true;
        request_out.type = candidate.type;
        request_out.pose_id = candidate.reference.pose_id;
        request_out.branch_id = candidate.reference.branch_id;
        request_out.confidence = candidate.confidence;
        return true;
    }

    bool stop_relocalize_branch_has_arrived(const motion_mcu_incoming_state::local_position_state &local_position)
    {
        if (stop_relocalize.pending_request.pending == false)
        {
            return false;
        }

        return local_position.branch_id != stop_relocalize.pending_request.candidate.reference.branch_id;
    }

    position_sensorfusion_anchors::reference_activation activate_stop_relocalize_request(std::uint32_t now_ms)
    {
        position_sensorfusion_anchors::reference_activation activation = {};

        if (stop_relocalize.pending_request.pending == false)
        {
            return activation;
        }

        activation.valid = true;
        activation.type = position_sensorfusion_anchors::anchor_type::position_only;
        activation.has_saved_global_heading = stop_relocalize.pending_request.has_saved_global_heading;
        activation.source_pose_id = stop_relocalize.pending_request.candidate.reference.pose_id;
        activation.source_branch_id = stop_relocalize.pending_request.candidate.reference.branch_id;
        activation.activation_time_ms = now_ms;
        activation.confidence = stop_relocalize.pending_request.candidate.confidence;
        activation.saved_global_heading_urad = stop_relocalize.pending_request.saved_global_heading_urad;
        activation.reference = stop_relocalize.pending_request.candidate.reference;
        stop_relocalize.pending_request = {};
        return activation;
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
        stop_relocalize_test_enabled = false;
        stop_relocalize = {};
        stop_relocalize.phase = stop_relocalize_phase::disabled;
        pure_pursuit::set_external_hold(false);
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

    void set_stop_relocalize_test_enabled(bool enabled)
    {
        stop_relocalize_test_enabled = enabled;
        reset_stop_relocalize_runtime(true);
        set_stop_relocalize_phase(enabled ? stop_relocalize_phase::monitoring : stop_relocalize_phase::disabled, 0U, 0U);
    }

    bool is_stop_relocalize_test_enabled()
    {
        return stop_relocalize_test_enabled;
    }

    position_sensorfusion_pipeline::stop_relocalize_status_snapshot read_stop_relocalize_status(std::uint32_t now_ms)
    {
        position_sensorfusion_pipeline::stop_relocalize_status_snapshot snapshot = {};
        snapshot.enabled = stop_relocalize_test_enabled;
        snapshot.active = stop_relocalize.active;
        snapshot.external_hold = pure_pursuit::is_external_hold_enabled();
        snapshot.phase = static_cast<std::uint8_t>(stop_relocalize.phase);
        snapshot.local_confidence_position = stop_relocalize.last_local_confidence_position;
        snapshot.filtered_confidence_position = stop_relocalize.last_filtered_confidence_position;
        snapshot.filtered_ready = stop_relocalize.last_filtered_ready;
        snapshot.filtered_age_ms = stop_relocalize.last_filtered_age_ms;
        snapshot.last_cancel_reason = static_cast<std::uint8_t>(stop_relocalize.last_cancel_reason);
        snapshot.phase_required_ms = stop_relocalize.phase_required_time_ms;
        snapshot.cooldown_remaining_ms = remaining_ms(now_ms, stop_relocalize.cooldown_until_ms);
        snapshot.request_pending = stop_relocalize.pending_request.pending;
        snapshot.request_pose_id = stop_relocalize.pending_request.candidate.reference.pose_id;
        snapshot.request_branch_id = stop_relocalize.pending_request.candidate.reference.branch_id;
        snapshot.collected_sample_count = stop_relocalize.last_collected_sample_count;
        snapshot.last_activation_performed = stop_relocalize.last_activation_performed;
        snapshot.last_activation_confidence = stop_relocalize.last_activation_confidence;
        snapshot.last_activation_time_ms = stop_relocalize.last_activation_time_ms;

        if (stop_relocalize.phase_start_time_ms != 0U)
        {
            snapshot.phase_elapsed_ms = position_sensorfusion_internal::elapsed_ms(now_ms, stop_relocalize.phase_start_time_ms);
        }

        if (stop_relocalize.attempt_start_time_ms != 0U)
        {
            snapshot.attempt_elapsed_ms = position_sensorfusion_internal::elapsed_ms(now_ms, stop_relocalize.attempt_start_time_ms);
        }

        return snapshot;
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
        position_sensorfusion_anchors::reference_activation stop_relocalize_activation = {};
        position_sensorfusion_anchors::branch_request stop_relocalize_request = {};
        stop_relocalize.last_activation_performed = false;
        stop_relocalize.last_local_confidence_position = local_position.confidence_position;
        stop_relocalize.last_filtered_confidence_position = filtered_position.confidence_position;
        stop_relocalize.last_filtered_age_ms =
            filtered_position.received_time_ms == 0U
                ? 0U
                : position_sensorfusion_internal::elapsed_ms(now_ms, filtered_position.received_time_ms);
        stop_relocalize.last_filtered_ready = filtered_position_ready_for_stop_relocalize(filtered_position, now_ms);

        const bool stop_relocalize_runnable = stop_relocalize_mode_should_run();

        if ((stop_relocalize_runnable == false) ||
            (local_position.has_pose == false) ||
            (current_reference.valid == false) ||
            (current_reference.has_heading == false))
        {
            reset_stop_relocalize_runtime(false);
            set_stop_relocalize_phase(stop_relocalize_runnable ? stop_relocalize_phase::blocked : stop_relocalize_phase::disabled, now_ms, 0U);
        }
        else
        {
            if ((stop_relocalize.active == false) && (now_ms < stop_relocalize.cooldown_until_ms))
            {
                set_stop_relocalize_phase(stop_relocalize_phase::cooldown, now_ms, 0U);
            }
            else if (stop_relocalize.active == false)
            {
                set_stop_relocalize_phase(stop_relocalize_phase::monitoring, now_ms, 0U);

                if (local_position.confidence_position <= stop_relocalize_stop_confidence)
                {
                    stop_relocalize.active = true;
                    stop_relocalize.attempt_start_time_ms = now_ms;
                    stop_relocalize.last_cancel_reason = stop_relocalize_cancel_reason::none;
                    begin_stop_relocalize_stationary_phase(
                        stop_relocalize_phase::holding_settle,
                        now_ms,
                        stop_relocalize_stationary_settle_time_ms,
                        local_position);
                }
            }

            if (stop_relocalize.active == true)
            {
                pure_pursuit::set_external_hold(true);

                if (stop_relocalize_attempt_timed_out(now_ms) == true)
                {
                    finish_stop_relocalize_without_activation(now_ms, stop_relocalize_cancel_reason::max_hold_timeout);
                }
                else if (stop_relocalize.pending_request.pending == true)
                {
                    set_stop_relocalize_phase(stop_relocalize_phase::request_wait_branch, now_ms, 0U);

                    if (stop_relocalize_branch_has_arrived(local_position) == true)
                    {
                        stop_relocalize_activation = activate_stop_relocalize_request(now_ms);
                        stop_relocalize.active = false;
                        stop_relocalize.cooldown_until_ms = now_ms + stop_relocalize_cooldown_ms;
                        stop_relocalize.last_activation_performed = true;
                        stop_relocalize.last_activation_confidence = stop_relocalize_activation.confidence;
                        stop_relocalize.last_activation_time_ms = now_ms;
                        set_stop_relocalize_phase(stop_relocalize_phase::cooldown, now_ms, 0U);
                        pure_pursuit::set_external_hold(false);
                        pure_pursuit_pipeline::notify_pose_reset();
                    }
                    else if (stop_relocalize_branch_request_timed_out(now_ms) == true)
                    {
                        finish_stop_relocalize_without_activation(now_ms, stop_relocalize_cancel_reason::branch_timeout);
                    }
                }
                else if (stop_relocalize.phase == stop_relocalize_phase::holding_settle)
                {
                    if (stop_relocalize_has_stood_still(local_position) == false)
                    {
                        begin_stop_relocalize_stationary_phase(
                            stop_relocalize_phase::holding_settle,
                            now_ms,
                            stop_relocalize_stationary_settle_time_ms,
                            local_position);
                    }
                    else if (position_sensorfusion_internal::elapsed_ms(now_ms, stop_relocalize.phase_start_time_ms) >= stop_relocalize_stationary_settle_time_ms)
                    {
                        begin_stop_relocalize_stationary_phase(
                            stop_relocalize_phase::holding_collect,
                            now_ms,
                            stop_relocalize_stationary_collect_time_ms,
                            local_position);
                    }
                }
                else if (stop_relocalize.phase == stop_relocalize_phase::holding_collect)
                {
                    if (stop_relocalize_has_stood_still(local_position) == false)
                    {
                        begin_stop_relocalize_stationary_phase(
                            stop_relocalize_phase::holding_settle,
                            now_ms,
                            stop_relocalize_stationary_settle_time_ms,
                            local_position);
                    }
                    else if (position_sensorfusion_internal::elapsed_ms(now_ms, stop_relocalize.phase_start_time_ms) >= stop_relocalize_stationary_collect_time_ms)
                    {
                        const position_sensorfusion_anchors::candidate candidate =
                            build_stop_relocalize_candidate(
                                stop_relocalize.phase_start_time_ms,
                                local_position.branch_id);

                        if (start_stop_relocalize_request(current_reference, candidate, now_ms, stop_relocalize_request) == true)
                        {
                            set_stop_relocalize_phase(stop_relocalize_phase::request_wait_branch, now_ms, 0U);
                        }
                        else
                        {
                            finish_stop_relocalize_without_activation(now_ms, stop_relocalize_cancel_reason::candidate_failed);
                        }
                    }
                }
            }
            else
            {
                pure_pursuit::set_external_hold(false);
            }
        }

        position_sensorfusion_anchors::candidate heading_candidate = {};
        position_sensorfusion_anchors::candidate position_candidate = {};

        if ((heading_anchor_enabled == true) && (stop_relocalize.active == false))
        {
            heading_candidate = heading_anchor::update();
        }

        if ((position_anchor_enabled == true) && (stop_relocalize.active == false))
        {
            position_candidate = position_anchor::update(current_reference.has_heading, current_reference.rotation_urad);
        }

        const position_sensorfusion_anchors::reference_activation entry_test_seed_activation = take_entry_seeded_drive_test_activation(local_position, now_ms);
        const position_sensorfusion_anchors::reference_activation mission_seed_activation = mission_reference_seed::update(local_position, now_ms);
        anchor_selector::output_snapshot anchor_output = {};

        if ((anchors_should_run() == true) && (local_only_mode == false) && (stop_relocalize.active == false) && (stop_relocalize_activation.valid == false))
        {
            anchor_output = anchor_selector::update(local_position, current_reference, heading_candidate, position_candidate, now_ms);
        }

        if ((stop_relocalize_request.valid == true) && (local_only_mode == false))
        {
            (void)position_correction_payload::send(stop_relocalize_request.pose_id, stop_relocalize_request.branch_id);
        }

        if ((anchor_output.request.valid == true) && (local_only_mode == false) && (stop_relocalize.active == false))
        {
            (void)position_correction_payload::send(anchor_output.request.pose_id, anchor_output.request.branch_id);
        }

        position_sensorfusion_anchors::reference_activation activation = anchor_output.activation;

        if (stop_relocalize_activation.valid == true)
        {
            activation = stop_relocalize_activation;
        }

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
        else if (stop_relocalize_activation.valid == true)
        {
            trace_event = build_trace_event_from_activation(position_trace_logger::event_action::activation, stop_relocalize_activation, local_position);
        }
        else if (stop_relocalize_request.valid == true)
        {
            trace_event = build_trace_event_from_request(
                stop_relocalize_request,
                current_reference,
                local_position,
                {},
                stop_relocalize.pending_request.candidate);
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
