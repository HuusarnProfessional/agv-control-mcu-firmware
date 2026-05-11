#include "alignment_anchor.hpp"

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
}

namespace alignment_anchor
{
    void clear(anchor_state &anchor)
    {
        anchor = {};
    }

    void set_local_branch_id(anchor_state &anchor, std::uint8_t branch_id)
    {
        anchor.local_branch_id = branch_id;
    }

    anchor_state create_initial(const motion_mcu_incoming_state::local_position_state &local_position, const global_position_heading::output_snapshot &global_position, std::uint16_t anchor_score)
    {
        anchor_state anchor = {};

        anchor.valid = true;
        anchor.global_x_um = global_position.x_um;
        anchor.global_y_um = global_position.y_um;
        anchor.local_x_um = local_position.x_um;
        anchor.local_y_um = local_position.y_um;
        anchor.local_heading_urad = local_position.heading_urad;
        anchor.confidence_position = anchor_score;
        anchor.local_branch_id = local_position.branch_id;
        anchor.global_sample_id = global_position.sample_id;

        if (global_position.has_heading == true)
        {
            anchor.has_heading = true;
            anchor.global_heading_urad = global_position.heading_urad;
            anchor.heading_offset_urad = normalize_angle_urad(global_position.heading_urad - local_position.heading_urad);
            anchor.confidence_heading = global_position.confidence_heading;
        }

        return anchor;
    }

    anchor_state create_branch(const global_position_heading::output_snapshot &global_position, std::uint16_t anchor_score)
    {
        anchor_state anchor = {};

        anchor.valid = true;
        anchor.global_x_um = global_position.x_um;
        anchor.global_y_um = global_position.y_um;
        anchor.local_x_um = 0;
        anchor.local_y_um = 0;
        anchor.local_heading_urad = 0;
        anchor.confidence_position = anchor_score;
        anchor.global_sample_id = global_position.sample_id;

        if (global_position.has_heading == true)
        {
            anchor.has_heading = true;
            anchor.global_heading_urad = global_position.heading_urad;
            anchor.heading_offset_urad = global_position.heading_urad;
            anchor.confidence_heading = global_position.confidence_heading;
        }

        return anchor;
    }
}