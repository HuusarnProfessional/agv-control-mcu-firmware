#include "mission_reference_seed.hpp"

#include <cmath>
#include <cstdint>

#include "../../mission/mission_runner.hpp"
#include "../../pure_pursuit/internal/path_accessor.hpp"

namespace
{
    constexpr std::uint16_t mission_reference_confidence = 700U;
    constexpr std::int32_t pi_urad = 3141593;
    constexpr std::int32_t two_pi_urad = 6283185;

    bool seed_pending = false;
    bool seed_used = false;

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

    std::int32_t calculate_heading_urad(const pure_pursuit_internal::path_point &first_point, const pure_pursuit_internal::path_point &second_point)
    {
        const double delta_x_mm = static_cast<double>(second_point.x_mm - first_point.x_mm);
        const double delta_y_mm = static_cast<double>(second_point.y_mm - first_point.y_mm);
        const double heading_rad = std::atan2(delta_y_mm, delta_x_mm);
        const std::int32_t heading_urad = static_cast<std::int32_t>(std::llround(heading_rad * 1000000.0));

        return normalize_angle_urad(heading_urad);
    }

    bool read_first_segment(pure_pursuit_internal::path_point &first_point_out, pure_pursuit_internal::path_point &second_point_out)
    {
        std::uint16_t current_part = 0U;
        const bool has_current_part = mission_runner::get_current_part(current_part);

        if (has_current_part == false)
        {
            return false;
        }

        if (current_part != 0U)
        {
            return false;
        }

        const bool has_first_point = pure_pursuit_internal::get_point(current_part, 0U, first_point_out);
        const bool has_second_point = pure_pursuit_internal::get_point(current_part, 1U, second_point_out);

        if (has_first_point == false)
        {
            return false;
        }

        if (has_second_point == false)
        {
            return false;
        }

        if ((first_point_out.x_mm == second_point_out.x_mm) && (first_point_out.y_mm == second_point_out.y_mm))
        {
            return false;
        }

        return true;
    }

    filtered_global_position::output_snapshot build_seed_global_reference(const pure_pursuit_internal::path_point &first_point, const pure_pursuit_internal::path_point &second_point, std::uint32_t now_ms)
    {
        filtered_global_position::output_snapshot global_reference = {};

        global_reference.has_position = true;
        global_reference.x_um = static_cast<std::int64_t>(first_point.x_mm) * 1000LL;
        global_reference.y_um = static_cast<std::int64_t>(first_point.y_mm) * 1000LL;
        global_reference.z_um = 0;
        global_reference.confidence_position = mission_reference_confidence;
        global_reference.has_heading = true;
        global_reference.heading_urad = calculate_heading_urad(first_point, second_point);
        global_reference.confidence_heading = mission_reference_confidence;
        global_reference.accepted = true;
        global_reference.raw_confidence_position = mission_reference_confidence;
        global_reference.history_confidence = mission_reference_confidence;
        global_reference.accepted_sample_count = 0U;
        global_reference.heading_sample_count = 2U;
        global_reference.sample_id = 0U;
        global_reference.request_id = 0U;
        global_reference.received_time_ms = now_ms;

        return global_reference;
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

    bool is_pending()
    {
        return seed_pending;
    }

    global_reference_selector::reference_activation update(const motion_mcu_incoming_state::local_position_state &local_position, const global_reference_selector::current_reference_snapshot &current_reference, std::uint32_t now_ms)
    {
        (void)current_reference;

        global_reference_selector::reference_activation activation = {};

        if (mission_runner::is_running() == false)
        {
            seed_pending = false;
            return activation;
        }

        if (seed_pending == false)
        {
            return activation;
        }

        if (seed_used == true)
        {
            return activation;
        }

        if (local_position.has_pose == false)
        {
            return activation;
        }

        pure_pursuit_internal::path_point first_point = {};
        pure_pursuit_internal::path_point second_point = {};
        const bool has_first_segment = read_first_segment(first_point, second_point);

        if (has_first_segment == false)
        {
            return activation;
        }

        activation.has_activation = true;
        activation.is_initial_reference = true;
        activation.is_mission_seed = true;
        activation.source_pose_id = local_position.pose_id;
        activation.source_branch_id = local_position.branch_id;
        activation.activation_time_ms = now_ms;
        activation.reference_score = mission_reference_confidence;
        activation.global_reference = build_seed_global_reference(first_point, second_point, now_ms);
        seed_pending = false;
        seed_used = true;

        return activation;
    }
}
