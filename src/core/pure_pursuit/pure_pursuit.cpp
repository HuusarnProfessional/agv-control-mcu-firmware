#include "pure_pursuit.hpp"

#include <Arduino.h>
#include <cmath>

#include "pure_pursuit_tuning.hpp"
#include "internal/math_helpers.hpp"
#include "internal/motion_output.hpp"
#include "internal/path_accessor.hpp"
#include "internal/target_evaluator.hpp"

namespace
{
    constexpr std::uint8_t stop_reason_none = 0U;
    constexpr std::uint8_t stop_reason_heartbeat = 1U;
    constexpr std::uint8_t stop_reason_no_pose = 2U;
    constexpr std::uint8_t stop_reason_goal_success = 3U;
    constexpr std::uint8_t stop_reason_goal_fail = 4U;
    constexpr std::uint8_t stop_reason_external_stop = 5U;

    struct closest_path_state
    {
        std::uint16_t segment_index = 0u;
        double segment_t = 0.0;
        double x_mm = 0.0;
        double y_mm = 0.0;
    };

    pure_pursuit::snapshot g_snapshot = {};

    void log_pass(const char *step_name)
    {
        Serial.print("pure_pursuit ");
        Serial.print(step_name);
        Serial.println(" pass");
    }

    void log_fail(const char *step_name)
    {
        Serial.print("pure_pursuit ");
        Serial.print(step_name);
        Serial.println(" fail");
    }

    void finish_path(bool success)
    {
        if (success == true)
        {
            g_snapshot.stop_reason = stop_reason_goal_success;
        }
        else
        {
            g_snapshot.stop_reason = stop_reason_goal_fail;
        }

        pure_pursuit_internal::send_stop_motion(g_snapshot);
        g_snapshot.active = false;
        g_snapshot.complete = true;
        g_snapshot.success = success;

        if (success == true)
        {
            log_pass("finish_path");
        }
        else
        {
            log_fail("finish_path");
        }
    }

    bool tick_validate_runtime(const position_sensorfusion::output_snapshot &local_position, const motion_mcu_heartbeat::snapshot &heartbeat)
    {
        if (heartbeat.packet_timed_out == true)
        {
            pure_pursuit_internal::send_stop_motion(g_snapshot);
            g_snapshot.active = false;
            g_snapshot.complete = true;
            g_snapshot.success = false;
            g_snapshot.stop_reason = stop_reason_heartbeat;
            log_fail("tick_validate_runtime heartbeat");
            return false;
        }

        if (local_position.has_pose == false)
        {
            pure_pursuit_internal::send_stop_motion(g_snapshot);
            g_snapshot.active = false;
            g_snapshot.complete = true;
            g_snapshot.success = false;
            g_snapshot.stop_reason = stop_reason_no_pose;
            log_fail("tick_validate_runtime local_position");
            return false;
        }

        return true;
    }

    closest_path_state find_closest_path_state(double x_mm, double y_mm, std::uint16_t start_index)
    {
        closest_path_state best = {};

        if (g_snapshot.point_count == 0u)
        {
            return best;
        }

        if (g_snapshot.point_count == 1u)
        {
            pure_pursuit_internal::path_point only_point = {};
            const bool has_point = pure_pursuit_internal::get_point(g_snapshot.part_number, 0u, only_point);

            if (has_point == false)
            {
                return best;
            }

            best.x_mm = static_cast<double>(only_point.x_mm);
            best.y_mm = static_cast<double>(only_point.y_mm);
            return best;
        }

        const std::uint16_t segment_start_index = start_index > 0u ? static_cast<std::uint16_t>(start_index - 1u) : 0u;
        double best_distance_sq = 0.0;
        bool has_best = false;

        std::uint16_t checked_segment_count = 0u;

        for (std::uint16_t index = segment_start_index; index + 1u < g_snapshot.point_count; ++index)
        {
            if (checked_segment_count >= pure_pursuit_tuning::k_mission_max_closest_segment_checks)
            {
                break;
            }

            ++checked_segment_count;
            pure_pursuit_internal::path_point point_a = {};
            pure_pursuit_internal::path_point point_b = {};
            const bool has_point_a = pure_pursuit_internal::get_point(g_snapshot.part_number, index, point_a);
            const bool has_point_b = pure_pursuit_internal::get_point(g_snapshot.part_number, static_cast<std::uint16_t>(index + 1u), point_b);

            if ((has_point_a == false) || (has_point_b == false))
            {
                continue;
            }

            const double ax = static_cast<double>(point_a.x_mm);
            const double ay = static_cast<double>(point_a.y_mm);
            const double bx = static_cast<double>(point_b.x_mm);
            const double by = static_cast<double>(point_b.y_mm);
            const double abx = bx - ax;
            const double aby = by - ay;
            const double ab_len_sq = (abx * abx) + (aby * aby);
            double segment_t = 0.0;

            if (ab_len_sq > 1e-9)
            {
                const double projected_t = (((x_mm - ax) * abx) + ((y_mm - ay) * aby)) / ab_len_sq;
                segment_t = pure_pursuit_internal::clamp_unit(projected_t);
            }

            const double px = ax + (abx * segment_t);
            const double py = ay + (aby * segment_t);
            const double dx = px - x_mm;
            const double dy = py - y_mm;
            const double distance_sq = (dx * dx) + (dy * dy);

            if ((has_best == false) || (distance_sq < best_distance_sq))
            {
                best_distance_sq = distance_sq;
                best.segment_index = index;
                best.segment_t = segment_t;
                best.x_mm = px;
                best.y_mm = py;
                has_best = true;
            }
        }

        return best;
    }

    std::uint16_t find_lookahead_target_from_progress(const closest_path_state &closest, double lookahead_mm, double &target_x_mm, double &target_y_mm)
    {
        if (g_snapshot.point_count == 0u)
        {
            target_x_mm = 0.0;
            target_y_mm = 0.0;
            return 0u;
        }

        if (g_snapshot.point_count == 1u)
        {
            pure_pursuit_internal::path_point only_point = {};
            const bool has_point = pure_pursuit_internal::get_point(g_snapshot.part_number, 0u, only_point);

            if (has_point == false)
            {
                target_x_mm = 0.0;
                target_y_mm = 0.0;
                return 0u;
            }

            target_x_mm = static_cast<double>(only_point.x_mm);
            target_y_mm = static_cast<double>(only_point.y_mm);
            return 0u;
        }

        double remaining_lookahead_mm = lookahead_mm;
        std::uint16_t segment_index = closest.segment_index;
        double segment_t = closest.segment_t;
        std::uint16_t lookahead_step_count = 0u;

        while (segment_index + 1u < g_snapshot.point_count)
        {
            if (lookahead_step_count >= pure_pursuit_tuning::k_mission_max_lookahead_segment_steps)
            {
                break;
            }

            ++lookahead_step_count;
            pure_pursuit_internal::path_point point_a = {};
            pure_pursuit_internal::path_point point_b = {};
            const bool has_point_a = pure_pursuit_internal::get_point(g_snapshot.part_number, segment_index, point_a);
            const bool has_point_b = pure_pursuit_internal::get_point(g_snapshot.part_number, static_cast<std::uint16_t>(segment_index + 1u), point_b);

            if ((has_point_a == false) || (has_point_b == false))
            {
                break;
            }

            const double ax = static_cast<double>(point_a.x_mm);
            const double ay = static_cast<double>(point_a.y_mm);
            const double bx = static_cast<double>(point_b.x_mm);
            const double by = static_cast<double>(point_b.y_mm);
            const double abx = bx - ax;
            const double aby = by - ay;
            const double segment_length_mm = std::sqrt((abx * abx) + (aby * aby));

            if (segment_length_mm <= 1e-9)
            {
                segment_index = static_cast<std::uint16_t>(segment_index + 1u);
                segment_t = 0.0;
                continue;
            }

            const double remaining_segment_length_mm = segment_length_mm * (1.0 - segment_t);

            if (remaining_lookahead_mm <= remaining_segment_length_mm)
            {
                const double advance_t = remaining_lookahead_mm / segment_length_mm;
                const double target_t = segment_t + advance_t;
                target_x_mm = ax + (abx * target_t);
                target_y_mm = ay + (aby * target_t);
                return static_cast<std::uint16_t>(segment_index + 1u);
            }

            remaining_lookahead_mm -= remaining_segment_length_mm;
            segment_index = static_cast<std::uint16_t>(segment_index + 1u);
            segment_t = 0.0;
        }

        pure_pursuit_internal::path_point last_point = {};
        const std::uint16_t last_index = static_cast<std::uint16_t>(g_snapshot.point_count - 1u);
        const bool has_last_point = pure_pursuit_internal::get_point(g_snapshot.part_number, last_index, last_point);

        if (has_last_point == false)
        {
            target_x_mm = 0.0;
            target_y_mm = 0.0;
            return 0u;
        }

        target_x_mm = static_cast<double>(last_point.x_mm);
        target_y_mm = static_cast<double>(last_point.y_mm);
        return last_index;
    }

    pure_pursuit_internal::driveable_target_state find_driveable_target_from_progress(const closest_path_state &closest, double robot_x_mm, double robot_y_mm, double heading_rad)
    {
        double fallback_x_mm = 0.0;
        double fallback_y_mm = 0.0;
        const std::uint16_t fallback_index = find_lookahead_target_from_progress(closest, pure_pursuit_tuning::k_mission_lookahead_mm, fallback_x_mm, fallback_y_mm);
        pure_pursuit_internal::driveable_target_state fallback = pure_pursuit_internal::evaluate_driveable_target(robot_x_mm, robot_y_mm, heading_rad, fallback_x_mm, fallback_y_mm, fallback_index);

        double scan_distance_mm = pure_pursuit_tuning::k_mission_driveable_target_scan_start_mm;

        for (std::uint8_t scan_index = 0u; scan_index < pure_pursuit_tuning::k_mission_driveable_target_scan_count; ++scan_index)
        {
            double candidate_x_mm = 0.0;
            double candidate_y_mm = 0.0;
            const std::uint16_t candidate_index = find_lookahead_target_from_progress(closest, scan_distance_mm, candidate_x_mm, candidate_y_mm);
            const pure_pursuit_internal::driveable_target_state candidate = pure_pursuit_internal::evaluate_driveable_target(robot_x_mm, robot_y_mm, heading_rad, candidate_x_mm, candidate_y_mm, candidate_index);

            if (candidate.valid == true)
            {
                return candidate;
            }

            scan_distance_mm += pure_pursuit_tuning::k_mission_driveable_target_scan_step_mm;
        }

        return fallback;
    }

    void write_target_snapshot(const pure_pursuit_internal::driveable_target_state &target, double heading_error_deg)
    {
        g_snapshot.lookahead_point_index = target.point_index;
        g_snapshot.target_valid = target.valid;
        g_snapshot.target_forward_ok = target.forward_ok;
        g_snapshot.target_curvature_ok = target.curvature_ok;
        g_snapshot.target_x_mm = target.x_mm;
        g_snapshot.target_y_mm = target.y_mm;
        g_snapshot.target_curvature = target.curvature;
        g_snapshot.target_distance_mm = std::sqrt(target.distance_sq_mm);
        g_snapshot.forward_mm = target.forward_mm;
        g_snapshot.left_mm = target.left_mm;
        g_snapshot.heading_error_deg = heading_error_deg;
    }

    void log_tracking_state(
        double robot_x_mm,
        double robot_y_mm,
        double heading_deg,
        const closest_path_state &closest,
        const pure_pursuit_internal::driveable_target_state &target)
    {
        Serial.print("pure_pursuit pose x_mm=");
        Serial.print(robot_x_mm, 1);
        Serial.print(" y_mm=");
        Serial.print(robot_y_mm, 1);
        Serial.print(" heading_deg=");
        Serial.println(heading_deg, 1);

        Serial.print("pure_pursuit closest idx=");
        Serial.print(static_cast<unsigned int>(closest.segment_index));
        Serial.print(" x_mm=");
        Serial.print(closest.x_mm, 1);
        Serial.print(" y_mm=");
        Serial.println(closest.y_mm, 1);

        Serial.print("pure_pursuit target idx=");
        Serial.print(static_cast<unsigned int>(target.point_index));
        Serial.print(" x_mm=");
        Serial.print(target.x_mm, 1);
        Serial.print(" y_mm=");
        Serial.print(target.y_mm, 1);
        Serial.print(" forward_mm=");
        Serial.print(target.forward_mm, 1);
        Serial.print(" left_mm=");
        Serial.print(target.left_mm, 1);
        Serial.print(" curvature=");
        Serial.print(target.curvature, 5);
        Serial.print(" valid=");
        Serial.print(target.valid ? 1 : 0);
        Serial.print(" forward_ok=");
        Serial.print(target.forward_ok ? 1 : 0);
        Serial.print(" curvature_ok=");
        Serial.println(target.curvature_ok ? 1 : 0);
    }
}

namespace pure_pursuit
{
    void tick(std::uint32_t now_ms, const position_sensorfusion::output_snapshot &local_position, const motion_mcu_heartbeat::snapshot &heartbeat)
    {
        (void)now_ms;

        if (g_snapshot.active == false)
        {
            return;
        }

        const bool runtime_ok = tick_validate_runtime(local_position, heartbeat);

        if (runtime_ok == false)
        {
            return;
        }

        if (g_snapshot.point_count == 0u)
        {
            finish_path(true);
            return;
        }

        pure_pursuit_internal::path_point goal_point = {};
        const bool has_goal_point = pure_pursuit_internal::get_point(g_snapshot.part_number, static_cast<std::uint16_t>(g_snapshot.point_count - 1u), goal_point);

        if (has_goal_point == false)
        {
            finish_path(false);
            return;
        }

        const double x_mm = static_cast<double>(local_position.x_um) / 1000.0;
        const double y_mm = static_cast<double>(local_position.y_um) / 1000.0;
        const double goal_dx = static_cast<double>(goal_point.x_mm) - x_mm;
        const double goal_dy = static_cast<double>(goal_point.y_mm) - y_mm;
        const double goal_distance_mm = std::sqrt((goal_dx * goal_dx) + (goal_dy * goal_dy));

        if (goal_distance_mm <= pure_pursuit_tuning::k_mission_goal_tolerance_mm)
        {
            finish_path(true);
            return;
        }

        const closest_path_state closest = find_closest_path_state(x_mm, y_mm, g_snapshot.closest_point_index);
        g_snapshot.closest_point_index = closest.segment_index;

        const double heading_rad = static_cast<double>(local_position.heading_urad) / 1000000.0;
        const pure_pursuit_internal::driveable_target_state target = find_driveable_target_from_progress(closest, x_mm, y_mm, heading_rad);
        const double dx = target.x_mm - x_mm;
        const double dy = target.y_mm - y_mm;
        const double heading_deg = heading_rad * 180.0 / pure_pursuit_tuning::k_pi;
        const double heading_error_deg = pure_pursuit_internal::normalize_angle_deg((std::atan2(dy, dx) * 180.0 / pure_pursuit_tuning::k_pi) - heading_deg);
        const double abs_heading_error_deg = std::fabs(heading_error_deg);

        g_snapshot.robot_x_mm = x_mm;
        g_snapshot.robot_y_mm = y_mm;
        g_snapshot.robot_heading_deg = heading_deg;
        g_snapshot.robot_pose_id = local_position.pose_id;
        g_snapshot.robot_branch_id = local_position.branch_id;
        write_target_snapshot(target, heading_error_deg);
        log_tracking_state(x_mm, y_mm, heading_deg, closest, target);

        std::int32_t linear_velocity_mm_s = 0;
        std::int32_t yaw_rate_mdeg_s = 0;

        if (abs_heading_error_deg >= pure_pursuit_tuning::k_mission_turn_only_heading_error_deg)
        {
            yaw_rate_mdeg_s = pure_pursuit_internal::compute_turn_only_yaw_rate_mdeg_s(target.forward_mm, target.left_mm, heading_error_deg);
        }
        else
        {
            linear_velocity_mm_s = pure_pursuit_tuning::k_mission_linear_speed_mm_s;
            pure_pursuit_internal::apply_heading_speed_slowdown(linear_velocity_mm_s, abs_heading_error_deg);

            if (target.distance_sq_mm > 1.0)
            {
                yaw_rate_mdeg_s = pure_pursuit_internal::compute_tracking_yaw_rate_mdeg_s(target.curvature, linear_velocity_mm_s);
                pure_pursuit_internal::apply_heading_p_yaw_correction(yaw_rate_mdeg_s, heading_error_deg);
                pure_pursuit_internal::apply_yaw_rate_speed_limit(linear_velocity_mm_s, yaw_rate_mdeg_s);
            }

            if (target.forward_mm < 0.0)
            {
                linear_velocity_mm_s = 0;
            }
        }

        Serial.print("pure_pursuit heading_error_deg=");
        Serial.println(heading_error_deg, 2);
        pure_pursuit_internal::send_motion_command(g_snapshot, linear_velocity_mm_s, yaw_rate_mdeg_s);
    }

    void init()
    {
        g_snapshot = {};
    }

    bool start_part(std::uint16_t part_number)
    {
        std::uint16_t point_count = 0u;
        const bool has_point_count = pure_pursuit_internal::get_point_count(part_number, point_count);

        if (has_point_count == false)
        {
            g_snapshot = {};
            g_snapshot.part_number = part_number;
            log_fail("start_part");
            return false;
        }

        g_snapshot = {};
        g_snapshot.active = true;
        g_snapshot.has_path = true;
        g_snapshot.stop_reason = stop_reason_none;
        g_snapshot.part_number = part_number;
        g_snapshot.point_count = point_count;
        g_snapshot.closest_point_index = 0u;
        g_snapshot.lookahead_point_index = 0u;
        const bool start_ok = g_snapshot.point_count > 0u;

        if (start_ok == true)
        {
            log_pass("start_part");
        }
        else
        {
            log_fail("start_part");
        }

        return start_ok;
    }

    void stop()
    {
        const std::uint8_t previous_stop_reason = g_snapshot.stop_reason;

        if (g_snapshot.active == true)
        {
            pure_pursuit_internal::send_stop_motion(g_snapshot);
        }

        g_snapshot = {};

        if (previous_stop_reason != stop_reason_none)
        {
            g_snapshot.stop_reason = previous_stop_reason;
        }
        else
        {
            g_snapshot.stop_reason = stop_reason_external_stop;
        }
    }

    snapshot read_snapshot()
    {
        return g_snapshot;
    }
}
