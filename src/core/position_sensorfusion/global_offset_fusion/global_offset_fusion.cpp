#include "global_offset_fusion.hpp"

#include <cmath>
#include <cstdint>

namespace
{
    constexpr std::int32_t pi_urad = 3141593;
    constexpr std::int32_t two_pi_urad = 6283185;

    struct offset_state
    {
        bool valid = false;
        std::int64_t x_um = 0;
        std::int64_t y_um = 0;
        std::uint16_t confidence_position = 0U;

        bool heading_valid = false;
        std::int32_t heading_urad = 0;
        std::uint16_t confidence_heading = 0U;

        std::uint32_t global_sample_id = 0U;
    };

    offset_state current_offset = {};
    global_offset_fusion::output_snapshot latest_output = {};

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

    std::uint16_t larger_confidence(std::uint16_t left, std::uint16_t right)
    {
        if (left > right)
        {
            return left;
        }

        return right;
    }

    std::uint16_t smaller_confidence(std::uint16_t left, std::uint16_t right)
    {
        if (left < right)
        {
            return left;
        }

        return right;
    }

    std::int64_t calculate_weighted_offset_step(std::int64_t difference_um, std::uint16_t local_confidence, std::uint16_t global_confidence)
    {
        const std::uint32_t total_confidence = static_cast<std::uint32_t>(local_confidence) + static_cast<std::uint32_t>(global_confidence);

        if (total_confidence == 0U)
        {
            return 0;
        }

        const std::int64_t weighted_difference = difference_um * static_cast<std::int64_t>(global_confidence);
        const std::int64_t step_um = weighted_difference / static_cast<std::int64_t>(total_confidence);

        return step_um;
    }

    std::int32_t calculate_weighted_heading_step(std::int32_t difference_urad, std::uint16_t local_confidence, std::uint16_t global_confidence)
    {
        const std::uint32_t total_confidence = static_cast<std::uint32_t>(local_confidence) + static_cast<std::uint32_t>(global_confidence);

        if (total_confidence == 0U)
        {
            return 0;
        }

        const std::int64_t weighted_difference = static_cast<std::int64_t>(difference_urad) * static_cast<std::int64_t>(global_confidence);
        const std::int64_t step_urad = weighted_difference / static_cast<std::int64_t>(total_confidence);

        return static_cast<std::int32_t>(step_urad);
    }

    bool global_position_should_update_offset(const global_position_history_filter::output_snapshot &global_position)
    {
        if (global_position.has_position == false)
        {
            return false;
        }

        if (global_position.is_new_sample == false)
        {
            return false;
        }

        if (global_position.rejected == true)
        {
            return false;
        }

        return true;
    }

    void update_position_offset(const local_position_alignment_to_global::output_snapshot &local_position, const global_position_history_filter::output_snapshot &global_position)
    {
        const std::int64_t measured_offset_x_um = global_position.x_um - local_position.x_um;
        const std::int64_t measured_offset_y_um = global_position.y_um - local_position.y_um;

        if (current_offset.valid == false)
        {
            current_offset.x_um = 0;
            current_offset.y_um = 0;
            current_offset.valid = true;
        }

        const std::int64_t difference_x_um = measured_offset_x_um - current_offset.x_um;
        const std::int64_t difference_y_um = measured_offset_y_um - current_offset.y_um;
        const std::int64_t step_x_um = calculate_weighted_offset_step(difference_x_um, local_position.confidence_position, global_position.confidence_position);
        const std::int64_t step_y_um = calculate_weighted_offset_step(difference_y_um, local_position.confidence_position, global_position.confidence_position);

        current_offset.x_um += step_x_um;
        current_offset.y_um += step_y_um;
        current_offset.confidence_position = larger_confidence(current_offset.confidence_position, global_position.confidence_position);
        current_offset.global_sample_id = global_position.sample_id;
    }

    void update_heading_offset(const local_position_alignment_to_global::output_snapshot &local_position, const global_position_history_filter::output_snapshot &global_position)
    {
        if (global_position.has_heading == false)
        {
            return;
        }

        const std::int32_t measured_heading_offset_urad = normalize_angle_urad(global_position.heading_urad - local_position.heading_urad);

        if (current_offset.heading_valid == false)
        {
            current_offset.heading_urad = 0;
            current_offset.heading_valid = true;
        }

        const std::int32_t difference_urad = normalize_angle_urad(measured_heading_offset_urad - current_offset.heading_urad);
        const std::int32_t step_urad = calculate_weighted_heading_step(difference_urad, local_position.confidence_heading, global_position.confidence_heading);

        current_offset.heading_urad = normalize_angle_urad(current_offset.heading_urad + step_urad);
        current_offset.confidence_heading = larger_confidence(current_offset.confidence_heading, global_position.confidence_heading);
    }

    void update_offset_from_global(const local_position_alignment_to_global::output_snapshot &local_position, const global_position_history_filter::output_snapshot &global_position)
    {
        if (local_position.has_pose == false)
        {
            return;
        }

        if (global_position_should_update_offset(global_position) == false)
        {
            return;
        }

        update_position_offset(local_position, global_position);
        update_heading_offset(local_position, global_position);
    }

    global_offset_fusion::output_snapshot build_from_local(const local_position_alignment_to_global::output_snapshot &local_position)
    {
        global_offset_fusion::output_snapshot output = {};

        if (local_position.has_pose == false)
        {
            return output;
        }

        output.has_pose = true;
        output.x_um = local_position.x_um;
        output.y_um = local_position.y_um;
        output.heading_urad = local_position.heading_urad;
        output.confidence_position = local_position.confidence_position;
        output.confidence_heading = local_position.confidence_heading;
        output.pose_id = local_position.pose_id;
        output.branch_id = local_position.branch_id;

        if (current_offset.valid == true)
        {
            output.x_um += current_offset.x_um;
            output.y_um += current_offset.y_um;
            output.confidence_position = larger_confidence(local_position.confidence_position, current_offset.confidence_position);
            output.has_offset = true;
        }

        if (current_offset.heading_valid == true)
        {
            output.heading_urad = normalize_angle_urad(local_position.heading_urad + current_offset.heading_urad);
            output.confidence_heading = larger_confidence(local_position.confidence_heading, current_offset.confidence_heading);
            output.has_heading_offset = true;
        }

        return output;
    }

    global_offset_fusion::output_snapshot build_from_global(const global_position_history_filter::output_snapshot &global_position)
    {
        global_offset_fusion::output_snapshot output = {};

        if (global_position.has_position == false)
        {
            return output;
        }

        output.has_pose = true;
        output.x_um = global_position.x_um;
        output.y_um = global_position.y_um;
        output.confidence_position = global_position.confidence_position;

        if (global_position.has_heading == true)
        {
            output.heading_urad = global_position.heading_urad;
            output.confidence_heading = global_position.confidence_heading;
        }

        return output;
    }
}

namespace global_offset_fusion
{
    void init()
    {
        current_offset = {};
        latest_output = {};
    }

    output_snapshot update(const local_position_alignment_to_global::output_snapshot &local_position, const global_position_history_filter::output_snapshot &global_position)
    {
        if (local_position.has_pose == true)
        {
            update_offset_from_global(local_position, global_position);
            latest_output = build_from_local(local_position);

            return latest_output;
        }

        if (global_position.has_position == true)
        {
            latest_output = build_from_global(global_position);

            return latest_output;
        }

        latest_output = {};

        return latest_output;
    }

    output_snapshot read_output()
    {
        return latest_output;
    }
}
