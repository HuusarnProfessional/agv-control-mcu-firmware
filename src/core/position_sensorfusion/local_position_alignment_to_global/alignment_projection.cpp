#include "alignment_projection.hpp"

#include "alignment_confidence.hpp"

#include <cmath>
#include <cstdint>

namespace
{
    constexpr std::int32_t pi_urad = 3141593;
    constexpr std::int32_t two_pi_urad = 6283185;

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

    void rotate_offset(const alignment_anchor::anchor_state &anchor, std::int64_t local_x_um, std::int64_t local_y_um, std::int64_t &global_x_um, std::int64_t &global_y_um)
    {
        global_x_um = local_x_um;
        global_y_um = local_y_um;

        if (anchor.has_heading == false)
        {
            return;
        }

        const double heading_offset_rad = static_cast<double>(anchor.heading_offset_urad) / 1000000.0;
        const double cos_heading = std::cos(heading_offset_rad);
        const double sin_heading = std::sin(heading_offset_rad);
        const double rotated_x_um = (static_cast<double>(local_x_um) * cos_heading) - (static_cast<double>(local_y_um) * sin_heading);
        const double rotated_y_um = (static_cast<double>(local_x_um) * sin_heading) + (static_cast<double>(local_y_um) * cos_heading);

        global_x_um = static_cast<std::int64_t>(rotated_x_um);
        global_y_um = static_cast<std::int64_t>(rotated_y_um);
    }
}

namespace alignment_projection
{
    local_position_alignment_to_global::output_snapshot project(const alignment_anchor::anchor_state &anchor, const motion_mcu_incoming_state::local_position_state &local_position)
    {
        local_position_alignment_to_global::output_snapshot output = {};

        if (local_position.has_pose == false)
        {
            return output;
        }

        if (anchor.valid == false)
        {
            return output;
        }

        const std::int64_t local_offset_x_um = local_position.x_um - anchor.local_x_um;
        const std::int64_t local_offset_y_um = local_position.y_um - anchor.local_y_um;

        std::int64_t global_offset_x_um = 0;
        std::int64_t global_offset_y_um = 0;

        rotate_offset(anchor, local_offset_x_um, local_offset_y_um, global_offset_x_um, global_offset_y_um);

        output.has_pose = true;
        output.x_um = anchor.global_x_um + global_offset_x_um;
        output.y_um = anchor.global_y_um + global_offset_y_um;
        output.pose_id = local_position.pose_id;
        output.branch_id = local_position.branch_id;
        output.has_global_anchor = true;
        output.has_heading_anchor = anchor.has_heading;
        output.confidence_position = alignment_confidence::multiply(anchor.confidence_position, local_position.confidence_position);

        if (anchor.has_heading == true)
        {
            output.heading_urad = normalize_angle_urad(local_position.heading_urad + anchor.heading_offset_urad);
            output.confidence_heading = alignment_confidence::multiply(anchor.confidence_heading, local_position.confidence_heading);

            return output;
        }

        output.heading_urad = local_position.heading_urad;
        output.confidence_heading = local_position.confidence_heading;

        return output;
    }
}