#include "math_helpers.hpp"

#include <cmath>

#include "../pure_pursuit_tuning.hpp"
#include "../../control/primitives/command_speed/command_speed_state.hpp"

namespace pure_pursuit_internal
{
    std::int32_t scale_mission_speed(std::int16_t base_speed_mm_s)
    {
        const std::uint16_t requested_speed_mm_s = command_speed_state::get_requested_speed_mm_s();
        const std::int32_t scaled_speed_mm_s = static_cast<std::int32_t>(base_speed_mm_s) * static_cast<std::int32_t>(requested_speed_mm_s) / static_cast<std::int32_t>(pure_pursuit_tuning::k_mission_speed_scale_reference_mm_s);

        if (scaled_speed_mm_s < 1)
        {
            return 1;
        }

        return scaled_speed_mm_s;
    }

    double clamp_unit(double value)
    {
        if (value < 0.0)
        {
            return 0.0;
        }

        if (value > 1.0)
        {
            return 1.0;
        }

        return value;
    }

    double normalize_angle_deg(double angle_deg)
    {
        double normalized_angle_deg = std::fmod(angle_deg, 360.0);

        if (normalized_angle_deg > 180.0)
        {
            normalized_angle_deg -= 360.0;
        }

        if (normalized_angle_deg < -180.0)
        {
            normalized_angle_deg += 360.0;
        }

        return normalized_angle_deg;
    }

    std::int32_t get_mission_linear_speed_mm_s()
    {
        return scale_mission_speed(pure_pursuit_tuning::k_mission_linear_speed_mm_s);
    }

    std::int32_t get_mission_min_linear_speed_mm_s()
    {
        return scale_mission_speed(pure_pursuit_tuning::k_mission_min_linear_speed_mm_s);
    }

    void apply_heading_speed_slowdown(std::int32_t &linear_velocity_mm_s, double abs_heading_error_deg)
    {
        if (abs_heading_error_deg <= pure_pursuit_tuning::k_mission_speed_slowdown_heading_error_deg)
        {
            return;
        }

        const double heading_window_deg = pure_pursuit_tuning::k_mission_turn_only_heading_error_deg - pure_pursuit_tuning::k_mission_speed_slowdown_heading_error_deg;

        if (heading_window_deg <= 0.0)
        {
            return;
        }

        const double blend = (abs_heading_error_deg - pure_pursuit_tuning::k_mission_speed_slowdown_heading_error_deg) / heading_window_deg;
        double clamped_blend = blend;

        if (clamped_blend < 0.0)
        {
            clamped_blend = 0.0;
        }

        if (clamped_blend > 1.0)
        {
            clamped_blend = 1.0;
        }

        const double max_speed = static_cast<double>(get_mission_linear_speed_mm_s());
        const double min_speed = static_cast<double>(get_mission_min_linear_speed_mm_s());
        const double scaled_speed = max_speed - ((max_speed - min_speed) * clamped_blend);
        linear_velocity_mm_s = static_cast<std::int32_t>(scaled_speed);
    }

    std::int32_t compute_tracking_yaw_rate_mdeg_s(double curvature, std::int32_t linear_velocity_mm_s)
    {
        const double max_curvature = pure_pursuit_tuning::k_mission_min_turn_radius_mm > 1.0 ? 1.0 / pure_pursuit_tuning::k_mission_min_turn_radius_mm : 0.0;
        double clamped_curvature = curvature;

        if (max_curvature > 0.0)
        {
            if (clamped_curvature > max_curvature)
            {
                clamped_curvature = max_curvature;
            }

            if (clamped_curvature < -max_curvature)
            {
                clamped_curvature = -max_curvature;
            }
        }

        const double yaw_rate_deg_s = (static_cast<double>(linear_velocity_mm_s) * clamped_curvature) * 180.0 / pure_pursuit_tuning::k_pi;
        double commanded_yaw_rate_mdeg_s = yaw_rate_deg_s * 1000.0;

        if (commanded_yaw_rate_mdeg_s > pure_pursuit_tuning::k_mission_max_yaw_rate_mdeg_s)
        {
            commanded_yaw_rate_mdeg_s = pure_pursuit_tuning::k_mission_max_yaw_rate_mdeg_s;
        }

        if (commanded_yaw_rate_mdeg_s < -pure_pursuit_tuning::k_mission_max_yaw_rate_mdeg_s)
        {
            commanded_yaw_rate_mdeg_s = -pure_pursuit_tuning::k_mission_max_yaw_rate_mdeg_s;
        }

        return static_cast<std::int32_t>(commanded_yaw_rate_mdeg_s);
    }

    void apply_heading_p_yaw_correction(std::int32_t &yaw_rate_mdeg_s, double heading_error_deg)
    {
        const double correction_mdeg_s =
            heading_error_deg * pure_pursuit_tuning::k_mission_heading_p_yaw_rate_mdeg_s_per_deg;

        double corrected_yaw_rate_mdeg_s =
            static_cast<double>(yaw_rate_mdeg_s) + correction_mdeg_s;

        if (corrected_yaw_rate_mdeg_s > pure_pursuit_tuning::k_mission_max_yaw_rate_mdeg_s)
        {
            corrected_yaw_rate_mdeg_s = pure_pursuit_tuning::k_mission_max_yaw_rate_mdeg_s;
        }

        if (corrected_yaw_rate_mdeg_s < -pure_pursuit_tuning::k_mission_max_yaw_rate_mdeg_s)
        {
            corrected_yaw_rate_mdeg_s = -pure_pursuit_tuning::k_mission_max_yaw_rate_mdeg_s;
        }

        yaw_rate_mdeg_s = static_cast<std::int32_t>(corrected_yaw_rate_mdeg_s);
    }

    void apply_yaw_rate_speed_limit(std::int32_t &linear_velocity_mm_s, std::int32_t yaw_rate_mdeg_s)
    {
        if (pure_pursuit_tuning::k_mission_max_yaw_rate_mdeg_s <= 0)
        {
            return;
        }

        const double abs_yaw_rate_mdeg_s = std::fabs(static_cast<double>(yaw_rate_mdeg_s));
        const double yaw_ratio = abs_yaw_rate_mdeg_s / static_cast<double>(pure_pursuit_tuning::k_mission_max_yaw_rate_mdeg_s);
        double clamped_yaw_ratio = yaw_ratio;

        if (clamped_yaw_ratio < 0.0)
        {
            clamped_yaw_ratio = 0.0;
        }

        if (clamped_yaw_ratio > 1.0)
        {
            clamped_yaw_ratio = 1.0;
        }

        const double max_speed = static_cast<double>(get_mission_linear_speed_mm_s());
        const double min_speed = static_cast<double>(get_mission_min_linear_speed_mm_s());
        const double yaw_limited_speed = max_speed - ((max_speed - min_speed) * clamped_yaw_ratio);
        const std::int32_t yaw_limited_speed_mm_s = static_cast<std::int32_t>(yaw_limited_speed);

        if (linear_velocity_mm_s > yaw_limited_speed_mm_s)
        {
            linear_velocity_mm_s = yaw_limited_speed_mm_s;
        }
    }

    std::int32_t compute_turn_only_yaw_rate_mdeg_s(double forward_mm, double left_mm, double heading_error_deg)
    {
        if ((forward_mm < 0.0) && (std::fabs(left_mm) > 1.0))
        {
            if (left_mm >= 0.0)
            {
                return pure_pursuit_tuning::k_mission_turn_only_yaw_rate_mdeg_s;
            }

            return -pure_pursuit_tuning::k_mission_turn_only_yaw_rate_mdeg_s;
        }

        if (heading_error_deg >= 0.0)
        {
            return pure_pursuit_tuning::k_mission_turn_only_yaw_rate_mdeg_s;
        }

        return -pure_pursuit_tuning::k_mission_turn_only_yaw_rate_mdeg_s;
    }
}
