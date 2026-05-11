#include "global_position_history_filter.hpp"

#include "global_position_history.hpp"
#include "global_position_history_gates.hpp"

namespace
{
    constexpr std::uint16_t full_confidence = 1000U;
    constexpr std::uint16_t minimum_confidence_to_store_in_history = 500U;

    global_position_history::history_state history = {};
    global_position_history_filter::output_snapshot latest_output = {};
    bool has_last_sample_id = false;
    std::uint32_t last_sample_id = 0U;

    std::uint16_t multiply_confidence(std::uint16_t left, std::uint16_t right)
    {
        const std::uint32_t multiplied = static_cast<std::uint32_t>(left) * static_cast<std::uint32_t>(right);
        const std::uint32_t scaled = multiplied / full_confidence;

        return static_cast<std::uint16_t>(scaled);
    }

    bool sample_is_new(const global_position_heading::output_snapshot &global_position)
    {
        if (has_last_sample_id == false)
        {
            return true;
        }

        if (global_position.sample_id == last_sample_id)
        {
            return false;
        }

        return true;
    }

    global_position_history::sample convert_sample(const global_position_heading::output_snapshot &global_position)
    {
        global_position_history::sample sample = {};

        sample.valid = true;
        sample.sample_id = global_position.sample_id;
        sample.received_time_ms = global_position.received_time_ms;
        sample.x_um = global_position.x_um;
        sample.y_um = global_position.y_um;
        sample.z_um = global_position.z_um;
        sample.confidence_position = global_position.confidence_position;

        return sample;
    }

    global_position_history_filter::output_snapshot build_rejected_output(const global_position_heading::output_snapshot &global_position, std::uint16_t history_confidence)
    {
        global_position_history_filter::output_snapshot output = {};

        output.has_position = false;
        output.original_confidence_position = global_position.confidence_position;
        output.history_confidence = history_confidence;
        output.is_new_sample = true;
        output.rejected = true;
        output.sample_id = global_position.sample_id;
        output.received_time_ms = global_position.received_time_ms;

        return output;
    }

    global_position_history_filter::output_snapshot build_accepted_output(const global_position_heading::output_snapshot &global_position, std::uint16_t history_confidence)
    {
        global_position_history_filter::output_snapshot output = {};

        output.has_position = true;
        output.x_um = global_position.x_um;
        output.y_um = global_position.y_um;
        output.z_um = global_position.z_um;
        output.original_confidence_position = global_position.confidence_position;
        output.history_confidence = history_confidence;
        output.confidence_position = multiply_confidence(global_position.confidence_position, history_confidence);
        output.has_heading = global_position.has_heading;
        output.heading_urad = global_position.heading_urad;
        output.confidence_heading = global_position.confidence_heading;
        output.is_new_sample = true;
        output.rejected = false;
        output.sample_id = global_position.sample_id;
        output.received_time_ms = global_position.received_time_ms;

        return output;
    }

    std::uint16_t calculate_history_confidence(const global_position_history::sample &new_sample)
    {
        const std::uint16_t physical_confidence = global_position_history_gates::physical_jump_confidence(history, new_sample);
        const std::uint16_t prediction_confidence = global_position_history_gates::prediction_confidence(history, new_sample);
        const std::uint16_t hampel_confidence = global_position_history_gates::hampel_speed_confidence(history, new_sample);
        const std::uint16_t history_confidence = global_position_history_gates::combine_gate_confidence(physical_confidence, prediction_confidence, hampel_confidence);

        return history_confidence;
    }
}

namespace global_position_history_filter
{
    void init()
    {
        global_position_history::clear(history);
        latest_output = {};
        has_last_sample_id = false;
        last_sample_id = 0U;
    }

    output_snapshot update(const global_position_heading::output_snapshot &global_position)
    {
        if (global_position.has_position == false)
        {
            return latest_output;
        }

        if (sample_is_new(global_position) == false)
        {
            latest_output.is_new_sample = false;

            return latest_output;
        }

        const global_position_history::sample new_sample = convert_sample(global_position);
        const std::uint16_t history_confidence = calculate_history_confidence(new_sample);

        last_sample_id = global_position.sample_id;
        has_last_sample_id = true;

        if (history_confidence == 0U)
        {
            latest_output = build_rejected_output(global_position, history_confidence);

            return latest_output;
        }

        latest_output = build_accepted_output(global_position, history_confidence);

        if (history_confidence >= minimum_confidence_to_store_in_history)
        {
            global_position_history::push(history, new_sample);
        }

        return latest_output;
    }

    output_snapshot read_output()
    {
        return latest_output;
    }
}
