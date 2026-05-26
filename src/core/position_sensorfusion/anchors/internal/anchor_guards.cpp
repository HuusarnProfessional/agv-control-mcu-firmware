#include "anchor_guards.hpp"

#include "../anchor_selector_tuning.hpp"
#include "../../internal/geometry_helpers.hpp"

namespace
{
    bool point_is_inside_safe_area(std::int64_t x_um, std::int64_t y_um)
    {
        if (x_um < anchor_selector_tuning::safe_min_x_um)
        {
            return false;
        }

        if (x_um > anchor_selector_tuning::safe_max_x_um)
        {
            return false;
        }

        if (y_um < anchor_selector_tuning::safe_min_y_um)
        {
            return false;
        }

        if (y_um > anchor_selector_tuning::safe_max_y_um)
        {
            return false;
        }

        return true;
    }

    std::uint16_t calculate_pose_age_steps(std::uint16_t current_pose_id, std::uint16_t reference_pose_id)
    {
        return static_cast<std::uint16_t>(current_pose_id - reference_pose_id);
    }

    bool project_candidate_to_current_pose(const motion_mcu_incoming_state::local_position_state &local_position,
                                           const position_sensorfusion_anchors::current_reference &current_reference,
                                           const position_sensorfusion_anchors::candidate &candidate,
                                           std::int64_t &x_um_out,
                                           std::int64_t &y_um_out,
                                           std::int32_t &rotation_urad_out)
    {
        if (candidate.reference.has_local_reference == false)
        {
            return false;
        }

        std::int32_t rotation_urad = current_reference.rotation_urad;

        if (candidate.type == position_sensorfusion_anchors::anchor_type::heading_transform)
        {
            if (candidate.reference.has_heading == false)
            {
                return false;
            }

            rotation_urad = position_sensorfusion_internal::normalize_angle_urad(candidate.reference.heading_urad - candidate.reference.local_heading_urad);
        }

        rotation_urad_out = rotation_urad;

        const std::int64_t local_delta_x_um = local_position.x_um - candidate.reference.local_x_um;
        const std::int64_t local_delta_y_um = local_position.y_um - candidate.reference.local_y_um;
        std::int64_t global_delta_x_um = 0;
        std::int64_t global_delta_y_um = 0;
        position_sensorfusion_internal::rotate_xy_um(local_delta_x_um, local_delta_y_um, rotation_urad, global_delta_x_um, global_delta_y_um);

        x_um_out = candidate.reference.x_um + global_delta_x_um;
        y_um_out = candidate.reference.y_um + global_delta_y_um;
        return true;
    }
}

namespace anchor_guards
{
    bool reference_is_inside_safe_area(const position_sensorfusion_anchors::candidate &candidate)
    {
        if (candidate.valid == false)
        {
            return false;
        }

        if (candidate.reference.valid == false)
        {
            return false;
        }

        return point_is_inside_safe_area(candidate.reference.x_um, candidate.reference.y_um);
    }

    bool candidate_pose_can_be_replayed(const motion_mcu_incoming_state::local_position_state &local_position, const position_sensorfusion_anchors::candidate &candidate)
    {
        if (local_position.has_pose == false)
        {
            return false;
        }

        if (candidate.reference.has_local_reference == false)
        {
            return false;
        }

        if (candidate.reference.branch_id != local_position.branch_id)
        {
            return false;
        }

        const std::uint16_t pose_age_steps = calculate_pose_age_steps(local_position.pose_id, candidate.reference.pose_id);

        if (pose_age_steps > anchor_selector_tuning::maximum_reference_pose_age_steps)
        {
            return false;
        }

        return true;
    }

    bool candidate_position_jump_is_safe(const motion_mcu_incoming_state::local_position_state &local_position, const position_sensorfusion_anchors::current_reference &current_reference, const position_sensorfusion_anchors::candidate &candidate)
    {
        const projected_candidate_snapshot projection = build_projected_candidate_snapshot(local_position, current_reference, candidate);

        if (projection.valid == false)
        {
            return current_reference.valid == false;
        }

        const std::int64_t jump_distance_um = position_sensorfusion_internal::calculate_distance_um(projection.jump_x_um, projection.jump_y_um);

        if (jump_distance_um > anchor_selector_tuning::maximum_anchor_position_jump_um)
        {
            return false;
        }

        return true;
    }

    bool heading_is_consistent(const position_sensorfusion_anchors::current_reference &current_reference, const position_sensorfusion_anchors::candidate &candidate)
    {
        if (candidate.type != position_sensorfusion_anchors::anchor_type::heading_transform)
        {
            return true;
        }

        if (current_reference.has_heading == false)
        {
            return true;
        }

        if (candidate.reference.has_heading == false)
        {
            return false;
        }

        const std::int32_t heading_delta_urad = position_sensorfusion_internal::absolute_angle_delta_urad(candidate.reference.heading_urad, current_reference.heading_urad);

        if (heading_delta_urad > anchor_selector_tuning::maximum_heading_delta_urad)
        {
            return false;
        }

        return true;
    }

    projected_candidate_snapshot build_projected_candidate_snapshot(const motion_mcu_incoming_state::local_position_state &local_position,
                                                                    const position_sensorfusion_anchors::current_reference &current_reference,
                                                                    const position_sensorfusion_anchors::candidate &candidate)
    {
        projected_candidate_snapshot snapshot = {};

        if (current_reference.valid == false)
        {
            return snapshot;
        }

        if (local_position.has_pose == false)
        {
            return snapshot;
        }

        if (candidate.reference.has_local_reference == false)
        {
            return snapshot;
        }

        if (candidate.reference.valid == false)
        {
            return snapshot;
        }

        if (candidate.valid == false)
        {
            return snapshot;
        }

        std::int32_t rotation_urad = 0;

        if (project_candidate_to_current_pose(local_position, current_reference, candidate, snapshot.projected_x_um, snapshot.projected_y_um, rotation_urad) == false)
        {
            return snapshot;
        }

        snapshot.valid = true;
        snapshot.rotation_urad = rotation_urad;
        snapshot.jump_x_um = snapshot.projected_x_um - current_reference.x_um;
        snapshot.jump_y_um = snapshot.projected_y_um - current_reference.y_um;
        return snapshot;
    }
}
