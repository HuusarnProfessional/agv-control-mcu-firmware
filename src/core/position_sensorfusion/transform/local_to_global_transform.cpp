#include "local_to_global_transform.hpp"

#include "../internal/confidence_math.hpp"
#include "../internal/geometry_helpers.hpp"

namespace
{
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
        std::uint32_t sample_id = 0U;
        bool is_mission_seed = false;
    };

    transform_state active_transform = {};
    local_to_global_transform::output_snapshot latest_output = {};

    bool activation_is_valid(const motion_mcu_incoming_state::local_position_state &local_position, const position_sensorfusion_anchors::reference_activation &activation)
    {
        if (local_position.has_pose == false)
        {
            return false;
        }

        if (activation.valid == false)
        {
            return false;
        }

        if (activation.reference.valid == false)
        {
            return false;
        }

        if (activation.type == position_sensorfusion_anchors::anchor_type::none)
        {
            return false;
        }

        if (activation.type == position_sensorfusion_anchors::anchor_type::position_only)
        {
            if (active_transform.valid == false)
            {
                return false;
            }

            return true;
        }

        return activation.reference.has_heading;
    }

    bool activate_transform(const motion_mcu_incoming_state::local_position_state &local_position, const position_sensorfusion_anchors::reference_activation &activation)
    {
        if (activation_is_valid(local_position, activation) == false)
        {
            return false;
        }

        const bool position_only_anchor = activation.type == position_sensorfusion_anchors::anchor_type::position_only;
        const std::int32_t previous_rotation_urad = active_transform.rotation_urad;
        const std::uint16_t previous_heading_confidence = active_transform.confidence_heading;

        active_transform.valid = true;
        active_transform.is_mission_seed = activation.is_mission_seed;

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

        active_transform.global_reference_x_um = activation.reference.x_um;
        active_transform.global_reference_y_um = activation.reference.y_um;

        if (position_only_anchor == true)
        {
            std::int32_t preserved_global_heading_urad = 0;

            if (activation.has_saved_global_heading == true)
            {
                preserved_global_heading_urad = activation.saved_global_heading_urad;
            }
            else
            {
                preserved_global_heading_urad =
                    position_sensorfusion_internal::normalize_angle_urad(
                        activation.reference.local_heading_urad + previous_rotation_urad);
            }

            active_transform.rotation_urad =
                position_sensorfusion_internal::normalize_angle_urad(
                    preserved_global_heading_urad - local_position.heading_urad);
            active_transform.global_reference_heading_urad = preserved_global_heading_urad;
            active_transform.confidence_heading = previous_heading_confidence;
        }
        else
        {
            // Heading anchors carry an absolute global heading estimate into the new branch.
            // The activation-time local heading belongs to the new branch frame, so the
            // old branch-local reference heading must not be reused here.
            const std::int32_t anchored_global_heading_urad = activation.reference.heading_urad;
            active_transform.rotation_urad =
                position_sensorfusion_internal::normalize_angle_urad(
                    anchored_global_heading_urad - local_position.heading_urad);
            active_transform.global_reference_heading_urad = anchored_global_heading_urad;
            active_transform.confidence_heading = activation.reference.confidence_heading;
        }

        active_transform.confidence_position = activation.reference.confidence_position;
        active_transform.branch_id = local_position.branch_id;
        active_transform.sample_id = activation.reference.sample_id;

        return true;
    }

    local_to_global_transform::output_snapshot project_local_position(const motion_mcu_incoming_state::local_position_state &local_position, bool transform_activated)
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
        output.rotation_urad = active_transform.rotation_urad;
        output.branch_matches = local_position.branch_id == active_transform.branch_id;

        if (output.branch_matches == false)
        {
            return output;
        }

        const std::int64_t local_delta_x_um = local_position.x_um - active_transform.local_reference_x_um;
        const std::int64_t local_delta_y_um = local_position.y_um - active_transform.local_reference_y_um;
        std::int64_t global_delta_x_um = 0;
        std::int64_t global_delta_y_um = 0;
        position_sensorfusion_internal::rotate_xy_um(local_delta_x_um, local_delta_y_um, active_transform.rotation_urad, global_delta_x_um, global_delta_y_um);

        output.has_pose = true;
        output.x_um = active_transform.global_reference_x_um + global_delta_x_um;
        output.y_um = active_transform.global_reference_y_um + global_delta_y_um;
        output.heading_urad = position_sensorfusion_internal::normalize_angle_urad(local_position.heading_urad + active_transform.rotation_urad);
        output.confidence_position = position_sensorfusion_internal::smaller_confidence(local_position.confidence_position, active_transform.confidence_position);
        output.confidence_heading = position_sensorfusion_internal::smaller_confidence(local_position.confidence_heading, active_transform.confidence_heading);
        output.pose_id = local_position.pose_id;
        output.branch_id = local_position.branch_id;
        output.reference_sample_id = active_transform.sample_id;
        output.is_mission_seed = active_transform.is_mission_seed;

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
    }

    output_snapshot update(const motion_mcu_incoming_state::local_position_state &local_position, const position_sensorfusion_anchors::reference_activation &activation)
    {
        const bool transform_activated = activate_transform(local_position, activation);
        latest_output = project_local_position(local_position, transform_activated);

        return latest_output;
    }

    output_snapshot read_output(const motion_mcu_incoming_state::local_position_state &local_position)
    {
        latest_output = project_local_position(local_position, false);
        return latest_output;
    }
}
