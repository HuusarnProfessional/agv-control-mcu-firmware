#include "target_evaluator.hpp"

#include <cmath>

#include "../pure_pursuit_tuning.hpp"

namespace pure_pursuit_internal
{
    driveable_target_state evaluate_driveable_target(double robot_x_mm, double robot_y_mm, double heading_rad, double target_x_mm, double target_y_mm, std::uint16_t point_index)
    {
        driveable_target_state out = {};
        out.point_index = point_index;
        out.x_mm = target_x_mm;
        out.y_mm = target_y_mm;

        const double dx = target_x_mm - robot_x_mm;
        const double dy = target_y_mm - robot_y_mm;
        const double cos_h = std::cos(heading_rad);
        const double sin_h = std::sin(heading_rad);

        out.forward_mm = cos_h * dx + sin_h * dy;
        out.left_mm = -sin_h * dx + cos_h * dy;
        out.distance_sq_mm = (dx * dx) + (dy * dy);

        if (out.distance_sq_mm > 1.0)
        {
            out.curvature = (2.0 * out.left_mm) / out.distance_sq_mm;
        }

        const double min_distance_sq = pure_pursuit_tuning::k_mission_min_driveable_target_distance_mm * pure_pursuit_tuning::k_mission_min_driveable_target_distance_mm;
        const double max_curvature = pure_pursuit_tuning::k_mission_min_turn_radius_mm > 1.0 ? 1.0 / pure_pursuit_tuning::k_mission_min_turn_radius_mm : 0.0;

        if (out.forward_mm >= pure_pursuit_tuning::k_mission_min_driveable_forward_mm)
        {
            out.forward_ok = true;
        }

        if ((max_curvature <= 0.0) || (std::fabs(out.curvature) <= max_curvature))
        {
            out.curvature_ok = true;
        }

        if (out.forward_ok == false)
        {
            return out;
        }

        if ((out.distance_sq_mm < min_distance_sq) && (std::fabs(out.curvature) > 1e-9))
        {
            return out;
        }

        if (out.curvature_ok == false)
        {
            return out;
        }

        out.valid = true;
        return out;
    }
}
