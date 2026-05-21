#include "huber_pca_fit.hpp"

#include "heading_anchor_tuning.hpp"
#include "../internal/anchor_confidence.hpp"
#include "../../internal/confidence_math.hpp"

#include <cmath>

namespace
{
    bool run_iteration(const filtered_global::sample *samples, std::uint8_t sample_count, std::uint16_t *weights, std::uint32_t *residuals_um, double &direction_x_out, double &direction_y_out)
    {
        double weight_sum = 0.0;
        double center_x = 0.0;
        double center_y = 0.0;

        for (std::uint8_t index = 0U; index < sample_count; index++)
        {
            const double weight = static_cast<double>(weights[index]);
            center_x += weight * static_cast<double>(samples[index].x_um);
            center_y += weight * static_cast<double>(samples[index].y_um);
            weight_sum += weight;
        }

        if (weight_sum <= 0.0)
        {
            return false;
        }

        center_x /= weight_sum;
        center_y /= weight_sum;

        double xx = 0.0;
        double xy = 0.0;
        double yy = 0.0;

        for (std::uint8_t index = 0U; index < sample_count; index++)
        {
            const double weight = static_cast<double>(weights[index]);
            const double delta_x = static_cast<double>(samples[index].x_um) - center_x;
            const double delta_y = static_cast<double>(samples[index].y_um) - center_y;
            xx += weight * delta_x * delta_x;
            xy += weight * delta_x * delta_y;
            yy += weight * delta_y * delta_y;
        }

        xx /= weight_sum;
        xy /= weight_sum;
        yy /= weight_sum;

        if ((xx + yy) <= 1.0)
        {
            return false;
        }

        const double angle_rad = 0.5 * std::atan2(2.0 * xy, xx - yy);
        const double direction_x = std::cos(angle_rad);
        const double direction_y = std::sin(angle_rad);

        for (std::uint8_t index = 0U; index < sample_count; index++)
        {
            const double delta_x = static_cast<double>(samples[index].x_um) - center_x;
            const double delta_y = static_cast<double>(samples[index].y_um) - center_y;
            const double residual_um = std::fabs((delta_x * direction_y) - (delta_y * direction_x));
            residuals_um[index] = static_cast<std::uint32_t>(std::llround(residual_um));

            std::uint32_t huber_weight = position_sensorfusion_internal::full_confidence;

            if (residuals_um[index] > heading_anchor_tuning::huber_delta_um)
            {
                huber_weight = heading_anchor_tuning::huber_delta_um * static_cast<std::uint32_t>(position_sensorfusion_internal::full_confidence) / residuals_um[index];
            }

            if (huber_weight == 0U)
            {
                huber_weight = 1U;
            }

            std::uint32_t position_weight = samples[index].confidence_position;

            if (position_weight == 0U)
            {
                position_weight = 1U;
            }

            std::uint32_t combined_weight = position_weight * huber_weight / static_cast<std::uint32_t>(position_sensorfusion_internal::full_confidence);

            if (combined_weight == 0U)
            {
                combined_weight = 1U;
            }

            if (combined_weight > position_sensorfusion_internal::full_confidence)
            {
                combined_weight = position_sensorfusion_internal::full_confidence;
            }

            weights[index] = static_cast<std::uint16_t>(combined_weight);
        }

        direction_x_out = direction_x;
        direction_y_out = direction_y;
        return true;
    }
}

namespace huber_pca_fit
{
    fit_result fit(const filtered_global::sample *samples, std::uint8_t sample_count)
    {
        fit_result result = {};

        if ((sample_count < heading_anchor_tuning::minimum_sample_count) || (sample_count > heading_anchor_tuning::maximum_sample_count))
        {
            return result;
        }

        std::uint16_t weights[heading_anchor_tuning::maximum_sample_count] = {};

        for (std::uint8_t index = 0U; index < sample_count; index++)
        {
            weights[index] = samples[index].confidence_position;

            if (weights[index] == 0U)
            {
                weights[index] = 1U;
            }
        }

        for (std::uint8_t iteration = 0U; iteration < heading_anchor_tuning::iteration_count; iteration++)
        {
            if (run_iteration(samples, sample_count, weights, result.residuals_um, result.direction_x, result.direction_y) == false)
            {
                return {};
            }
        }

        result.valid = true;
        result.median_residual_um = anchor_confidence::median_residual_um(result.residuals_um, sample_count);

        return result;
    }
}
