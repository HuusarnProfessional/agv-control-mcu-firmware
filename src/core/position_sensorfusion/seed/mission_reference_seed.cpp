#include "mission_reference_seed.hpp"

#include "mission_reference_seed_tuning.hpp"
#include "../../mission/mission_runner.hpp"
#include "../../pure_pursuit/internal/path_accessor.hpp"
#include "../internal/geometry_helpers.hpp"

#include <cmath>

namespace
{
    bool seed_pending = false;
    bool seed_used = false;

    bool read_first_segment(pure_pursuit_internal::path_point &first_point_out, pure_pursuit_internal::path_point &second_point_out)
    {
        std::uint16_t current_part = 0U;

        if (mission_runner::get_current_part(current_part) == false)
        {
            return false;
        }

        if (current_part != 0U)
        {
            return false;
        }

        if (pure_pursuit_internal::get_point(current_part, 0U, first_point_out) == false)
        {
            return false;
        }

        if (pure_pursuit_internal::get_point(current_part, 1U, second_point_out) == false)
        {
            return false;
        }

        if ((first_point_out.x_mm == second_point_out.x_mm) && (first_point_out.y_mm == second_point_out.y_mm))
        {
            return false;
        }

        return true;
    }

    std::int32_t calculate_heading_urad(const pure_pursuit_internal::path_point &first_point, const pure_pursuit_internal::path_point &second_point)
    {
        const double delta_x_mm = static_cast<double>(second_point.x_mm - first_point.x_mm);
        const double delta_y_mm = static_cast<double>(second_point.y_mm - first_point.y_mm);
        const double heading_rad = std::atan2(delta_y_mm, delta_x_mm);

        return position_sensorfusion_internal::normalize_angle_urad(static_cast<std::int32_t>(std::llround(heading_rad * 1000000.0)));
    }

    position_sensorfusion_anchors::global_reference build_reference(const pure_pursuit_internal::path_point &first_point, const pure_pursuit_internal::path_point &second_point, std::uint32_t now_ms)
    {
        position_sensorfusion_anchors::global_reference reference = {};
        reference.valid = true;
        reference.has_heading = true;
        reference.x_um = static_cast<std::int64_t>(first_point.x_mm) * 1000LL;
        reference.y_um = static_cast<std::int64_t>(first_point.y_mm) * 1000LL;
        reference.heading_urad = calculate_heading_urad(first_point, second_point);
        reference.confidence_position = mission_reference_seed_tuning::seed_confidence;
        reference.confidence_heading = mission_reference_seed_tuning::seed_confidence;
        reference.received_time_ms = now_ms;

        return reference;
    }
}

namespace mission_reference_seed
{
    void init()
    {
        seed_pending = false;
        seed_used = false;
    }

    void reset_runtime_state()
    {
        seed_pending = true;
        seed_used = false;
    }

    position_sensorfusion_anchors::reference_activation update(const motion_mcu_incoming_state::local_position_state &local_position, std::uint32_t now_ms)
    {
        position_sensorfusion_anchors::reference_activation activation = {};

        if (mission_runner::is_running() == false)
        {
            seed_pending = false;
            return activation;
        }

        if ((seed_pending == false) || (seed_used == true))
        {
            return activation;
        }

        if (local_position.has_pose == false)
        {
            return activation;
        }

        pure_pursuit_internal::path_point first_point = {};
        pure_pursuit_internal::path_point second_point = {};

        if (read_first_segment(first_point, second_point) == false)
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
        activation.confidence = mission_reference_seed_tuning::seed_confidence;
        activation.reference = build_reference(first_point, second_point, now_ms);
        seed_pending = false;
        seed_used = true;

        return activation;
    }
}
