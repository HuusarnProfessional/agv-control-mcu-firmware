#pragma once

#include <cstdint>

namespace pure_pursuit_internal
{
    struct driveable_target_state
    {
        bool valid = false;
        bool forward_ok = false;
        bool curvature_ok = false;
        std::uint16_t point_index = 0u;
        double x_mm = 0.0;
        double y_mm = 0.0;
        double forward_mm = 0.0;
        double left_mm = 0.0;
        double distance_sq_mm = 0.0;
        double curvature = 0.0;
    };

    driveable_target_state evaluate_driveable_target(double robot_x_mm, double robot_y_mm, double heading_rad, double target_x_mm, double target_y_mm, std::uint16_t point_index);
}
