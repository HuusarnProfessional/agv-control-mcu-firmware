#include "global_position_heading.hpp"

#include <cmath>
#include <cstdint>

namespace
{
    constexpr std::int64_t minimum_heading_distance_um = 50000;

    struct stored_sample
    {
        bool valid = false;
        std::uint32_t sample_id = 0U;
        std::uint32_t received_time_ms = 0U;
        std::int64_t x_um = 0;
        std::int64_t y_um = 0;
        std::int64_t z_um = 0;
        std::uint8_t quality_factor = 0U;
    };

    stored_sample previous_sample = {};
    stored_sample current_sample = {};

    bool has_last_sample_id = false;
    std::uint32_t last_sample_id = 0U;

    std::uint16_t quality_factor_to_confidence(std::uint8_t quality_factor)
    {
        return static_cast<std::uint16_t>(quality_factor) * 10U;
    }

    std::uint16_t smaller_confidence(std::uint16_t left, std::uint16_t right)
    {
        if (left < right)
        {
            return left;
        }

        return right;
    }

    std::int64_t calculate_distance_um(std::int64_t delta_x_um, std::int64_t delta_y_um)
    {
        const double delta_x = static_cast<double>(delta_x_um);
        const double delta_y = static_cast<double>(delta_y_um);
        const double distance = std::sqrt((delta_x * delta_x) + (delta_y * delta_y));

        return static_cast<std::int64_t>(distance);
    }

    bool sample_is_valid(const global_position_api::global_position_sample &sample)
    {
        if (sample.valid == false)
        {
            return false;
        }

        if (sample.status != global_position_api::global_position_status::ok)
        {
            return false;
        }

        return true;
    }

    bool sample_is_new(const global_position_api::global_position_sample &sample)
    {
        if (has_last_sample_id == false)
        {
            return true;
        }

        if (sample.sample_id == last_sample_id)
        {
            return false;
        }

        return true;
    }

    stored_sample convert_sample(const global_position_api::global_position_sample &sample)
    {
        stored_sample converted = {};

        converted.valid = true;
        converted.sample_id = sample.sample_id;
        converted.received_time_ms = sample.received_time_ms;
        converted.x_um = static_cast<std::int64_t>(sample.x_mm) * 1000;
        converted.y_um = static_cast<std::int64_t>(sample.y_mm) * 1000;
        converted.z_um = static_cast<std::int64_t>(sample.z_mm) * 1000;
        converted.quality_factor = sample.quality_factor;

        return converted;
    }

    void store_sample(const global_position_api::global_position_sample &sample)
    {
        previous_sample = current_sample;
        current_sample = convert_sample(sample);
        last_sample_id = sample.sample_id;
        has_last_sample_id = true;
    }

    std::int32_t calculate_heading_urad(std::int64_t delta_x_um, std::int64_t delta_y_um)
    {
        const double heading_rad = std::atan2(static_cast<double>(delta_y_um), static_cast<double>(delta_x_um));
        const double heading_urad = heading_rad * 1000000.0;

        return static_cast<std::int32_t>(heading_urad);
    }

    global_position_heading::output_snapshot build_output()
    {
        global_position_heading::output_snapshot output = {};

        if (current_sample.valid == false)
        {
            return output;
        }

        const std::uint16_t current_position_confidence = quality_factor_to_confidence(current_sample.quality_factor);

        output.has_position = true;
        output.x_um = current_sample.x_um;
        output.y_um = current_sample.y_um;
        output.z_um = current_sample.z_um;
        output.confidence_position = current_position_confidence;
        output.sample_id = current_sample.sample_id;
        output.received_time_ms = current_sample.received_time_ms;

        if (previous_sample.valid == false)
        {
            return output;
        }

        const std::int64_t delta_x_um = current_sample.x_um - previous_sample.x_um;
        const std::int64_t delta_y_um = current_sample.y_um - previous_sample.y_um;
        const std::int64_t distance_um = calculate_distance_um(delta_x_um, delta_y_um);

        if (distance_um < minimum_heading_distance_um)
        {
            return output;
        }

        const std::uint16_t previous_position_confidence = quality_factor_to_confidence(previous_sample.quality_factor);
        const std::uint16_t heading_confidence = smaller_confidence(previous_position_confidence, current_position_confidence);

        if (heading_confidence == 0U)
        {
            return output;
        }

        output.has_heading = true;
        output.heading_urad = calculate_heading_urad(delta_x_um, delta_y_um);
        output.confidence_heading = heading_confidence;

        return output;
    }
}

namespace global_position_heading
{
    void init()
    {
        previous_sample = {};
        current_sample = {};
        has_last_sample_id = false;
        last_sample_id = 0U;
    }

    output_snapshot update(const global_position_api::global_position_sample &sample)
    {
        if (sample_is_valid(sample) == true)
        {
            if (sample_is_new(sample) == true)
            {
                store_sample(sample);
            }
        }

        return build_output();
    }

    output_snapshot read_output()
    {
        return build_output();
    }
}
