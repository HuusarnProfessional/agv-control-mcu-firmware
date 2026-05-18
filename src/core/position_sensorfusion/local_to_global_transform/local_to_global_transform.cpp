#include "local_to_global_transform.hpp"

#include <cmath>
#include <cstdint>

namespace
{
    constexpr std::uint16_t full_confidence = 1000U;
    constexpr std::int32_t pi_urad = 3141593;
    constexpr std::int32_t two_pi_urad = 6283185;
    struct transform_state
    {
        bool valid = false;

        std::int64_t local_reference_x_um = 0;
        std::int64_t local_reference_y_um = 0;
        std::int32_t local_reference_heading_urad = 0;

        std::int64_t global_reference_x_um = 0;
        std::int64_t global_reference_y_um = 0;
        std::int32_t global_reference_heading_urad = 0;

        std::int32_t rotation_urad = 0;

        std::uint16_t confidence_position = 0U;
        std::uint16_t confidence_heading = 0U;

        std::uint8_t branch_id = 0U;
        std::uint32_t reference_sample_id = 0U;
        bool is_mission_seed = false;
        std::uint32_t activation_time_ms = 0U;
    };

    transform_state active_transform = {};
    local_to_global_transform::output_snapshot latest_output = {};
    local_to_global_transform::anchor_event_snapshot latest_anchor_event = {};
    std::uint32_t next_anchor_event_id = 1U;

    std::int32_t normalize_angle_urad(std::int32_t angle_urad)
    {
        const double normalized_double = std::remainder(static_cast<double>(angle_urad), static_cast<double>(two_pi_urad));
        std::int32_t normalized = static_cast<std::int32_t>(normalized_double);

        if (normalized > pi_urad)
        {
            normalized -= two_pi_urad;
        }

        if (normalized < -pi_urad)
        {
            normalized += two_pi_urad;
        }

        return normalized;
    }

    void rotate_local_delta(std::int64_t local_delta_x_um, std::int64_t local_delta_y_um, std::int32_t rotation_urad, std::int64_t &global_delta_x_um, std::int64_t &global_delta_y_um)
    {
        const double rotation_rad = static_cast<double>(rotation_urad) / 1000000.0;
        const double cos_rotation = std::cos(rotation_rad);
        const double sin_rotation = std::sin(rotation_rad);
        const double rotated_x_um = (static_cast<double>(local_delta_x_um) * cos_rotation) - (static_cast<double>(local_delta_y_um) * sin_rotation);
        const double rotated_y_um = (static_cast<double>(local_delta_x_um) * sin_rotation) + (static_cast<double>(local_delta_y_um) * cos_rotation);

        global_delta_x_um = static_cast<std::int64_t>(rotated_x_um);
        global_delta_y_um = static_cast<std::int64_t>(rotated_y_um);
    }

    std::int32_t calculate_matrix_value_ppm(double value)
    {
        return static_cast<std::int32_t>(std::llround(value * 1000000.0));
    }

    void store_anchor_event(const global_reference_selector::reference_activation &activation)
    {
        const double rotation_rad = static_cast<double>(active_transform.rotation_urad) / 1000000.0;
        const double cos_rotation = std::cos(rotation_rad);
        const double sin_rotation = std::sin(rotation_rad);

        latest_anchor_event.valid = true;
        latest_anchor_event.event_id = next_anchor_event_id;
        latest_anchor_event.is_initial_reference = activation.is_initial_reference;
        latest_anchor_event.is_mission_seed = activation.is_mission_seed;
        latest_anchor_event.source_pose_id = activation.source_pose_id;
        latest_anchor_event.source_branch_id = activation.source_branch_id;
        latest_anchor_event.activation_time_ms = activation.activation_time_ms;
        latest_anchor_event.reference_confidence = activation.reference_confidence;
        latest_anchor_event.reference_sample_id = active_transform.reference_sample_id;
        latest_anchor_event.local_reference_x_um = active_transform.local_reference_x_um;
        latest_anchor_event.local_reference_y_um = active_transform.local_reference_y_um;
        latest_anchor_event.local_reference_heading_urad = active_transform.local_reference_heading_urad;
        latest_anchor_event.global_reference_x_um = active_transform.global_reference_x_um;
        latest_anchor_event.global_reference_y_um = active_transform.global_reference_y_um;
        latest_anchor_event.global_reference_heading_urad = active_transform.global_reference_heading_urad;
        latest_anchor_event.rotation_urad = active_transform.rotation_urad;
        latest_anchor_event.matrix_cos_ppm = calculate_matrix_value_ppm(cos_rotation);
        latest_anchor_event.matrix_sin_ppm = calculate_matrix_value_ppm(sin_rotation);
        next_anchor_event_id++;
    }

    bool activate_transform(const motion_mcu_incoming_state::local_position_state &local_position, const global_reference_selector::reference_activation &activation)
    {
        if (local_position.has_pose == false)
        {
            return false;
        }

        if (activation.has_activation == false)
        {
            return false;
        }

        if (activation.global_reference.has_position == false)
        {
            return false;
        }

        if (activation.global_reference.has_heading == false)
        {
            return false;
        }

        active_transform.valid = true;

        if ((activation.is_initial_reference == true) || (activation.is_mission_seed == true))
        {
            active_transform.local_reference_x_um = local_position.x_um;
            active_transform.local_reference_y_um = local_position.y_um;
            active_transform.local_reference_heading_urad = local_position.heading_urad;
        }
        else
        {
            active_transform.local_reference_x_um = 0;
            active_transform.local_reference_y_um = 0;
            active_transform.local_reference_heading_urad = 0;
        }

        active_transform.global_reference_x_um = activation.global_reference.x_um;
        active_transform.global_reference_y_um = activation.global_reference.y_um;
        active_transform.global_reference_heading_urad = activation.global_reference.heading_urad;
        active_transform.rotation_urad = normalize_angle_urad(activation.global_reference.heading_urad - active_transform.local_reference_heading_urad);
        active_transform.confidence_position = activation.global_reference.confidence_position;
        active_transform.confidence_heading = activation.global_reference.confidence_heading;
        active_transform.branch_id = local_position.branch_id;
        active_transform.reference_sample_id = activation.global_reference.sample_id;
        active_transform.is_mission_seed = activation.is_mission_seed;
        active_transform.activation_time_ms = activation.activation_time_ms;

        store_anchor_event(activation);

        return true;
    }

    local_to_global_transform::output_snapshot project_local_position(const motion_mcu_incoming_state::local_position_state &local_position, bool transform_activated, std::uint32_t now_ms)
    {
        local_to_global_transform::output_snapshot output = {};

        output.transform_activated = transform_activated;

        if (local_position.has_pose == false)
        {
            return output;
        }

        if (active_transform.valid == false)
        {
            return output;
        }

        output.has_transform = true;
        output.branch_matches = local_position.branch_id == active_transform.branch_id;

        if (output.branch_matches == false)
        {
            return output;
        }

        const std::int64_t local_delta_x_um = local_position.x_um - active_transform.local_reference_x_um;
        const std::int64_t local_delta_y_um = local_position.y_um - active_transform.local_reference_y_um;
        std::int64_t global_delta_x_um = 0;
        std::int64_t global_delta_y_um = 0;

        rotate_local_delta(local_delta_x_um, local_delta_y_um, active_transform.rotation_urad, global_delta_x_um, global_delta_y_um);

        output.has_pose = true;
        output.x_um = active_transform.global_reference_x_um + global_delta_x_um;
        output.y_um = active_transform.global_reference_y_um + global_delta_y_um;
        output.heading_urad = normalize_angle_urad(local_position.heading_urad + active_transform.rotation_urad);
        output.confidence_position = local_position.confidence_position;
        output.confidence_heading = local_position.confidence_heading;
        output.pose_id = local_position.pose_id;
        output.branch_id = local_position.branch_id;
        output.reference_sample_id = active_transform.reference_sample_id;
        output.transform_confidence_position = full_confidence;
        output.transform_confidence_heading = full_confidence;
        output.is_mission_seed = active_transform.is_mission_seed;
        output.activation_time_ms = active_transform.activation_time_ms;

        return output;
    }
}

namespace local_to_global_transform
{
    void init()
    {
        reset_runtime_state();
    }

    void reset_runtime_state()
    {
        active_transform = {};
        latest_output = {};
        latest_anchor_event = {};
        next_anchor_event_id = 1U;
    }

    output_snapshot update(const motion_mcu_incoming_state::local_position_state &local_position, const global_reference_selector::reference_activation &activation, std::uint32_t now_ms)
    {
        bool transform_activated = false;

        if (activation.has_activation == true)
        {
            transform_activated = activate_transform(local_position, activation);
        }

        latest_output = project_local_position(local_position, transform_activated, now_ms);

        return latest_output;
    }

    output_snapshot read_output(const motion_mcu_incoming_state::local_position_state &local_position, std::uint32_t now_ms)
    {
        latest_output = project_local_position(local_position, false, now_ms);

        return latest_output;
    }

    anchor_event_snapshot read_anchor_event()
    {
        return latest_anchor_event;
    }
}
