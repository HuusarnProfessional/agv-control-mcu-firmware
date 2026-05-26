#include "filtered_global.hpp"

#include "filtered_global_tuning.hpp"
#include "internal/adaptive_low_pass.hpp"
#include "internal/bootstrap_filter.hpp"
#include "internal/history_buffer.hpp"
#include "internal/history_filter.hpp"
#include "../internal/confidence_math.hpp"
#include "../internal/geometry_helpers.hpp"

namespace
{
    filtered_global_history::state history = {};
    filtered_global::sample bootstrap_samples[filtered_global_tuning::bootstrap_max_sample_count] = {};
    std::uint8_t bootstrap_sample_count = 0U;
    filtered_global::output_snapshot latest_output = {};
    std::uint8_t stuck_rejected_sample_count = 0U;

    void reset_bootstrap_samples()
    {
        for (std::uint8_t index = 0U; index < filtered_global_tuning::bootstrap_max_sample_count; index++)
        {
            bootstrap_samples[index] = {};
        }

        bootstrap_sample_count = 0U;
    }

    void reset_runtime_state()
    {
        filtered_global_history::reset(history);
        reset_bootstrap_samples();
        latest_output = {};
        stuck_rejected_sample_count = 0U;
    }

    void push_bootstrap_sample(const filtered_global::sample &sample)
    {
        const std::uint8_t last_index = filtered_global_tuning::bootstrap_max_sample_count - 1U;

        for (std::uint8_t index = last_index; index > 0U; index--)
        {
            bootstrap_samples[index] = bootstrap_samples[index - 1U];
        }

        bootstrap_samples[0U] = sample;

        if (bootstrap_sample_count < filtered_global_tuning::bootstrap_max_sample_count)
        {
            bootstrap_sample_count++;
        }
    }

    filtered_global::output_snapshot build_output_from_sample(const filtered_global::sample &sample, std::uint32_t now_ms)
    {
        filtered_global::output_snapshot output = latest_output;

        if (sample.valid == false)
        {
            return output;
        }

        const std::uint32_t position_age_ms = position_sensorfusion_internal::elapsed_ms(now_ms, sample.received_time_ms);
        const std::uint16_t age_confidence = position_sensorfusion_internal::age_to_confidence(position_age_ms, filtered_global_tuning::position_confidence_full_age_ms, filtered_global_tuning::position_confidence_zero_age_ms);

        output.has_position = true;
        output.x_um = sample.x_um;
        output.y_um = sample.y_um;
        output.z_um = sample.z_um;
        output.confidence_position = position_sensorfusion_internal::smaller_confidence(sample.confidence_position, age_confidence);
        output.sample_id = sample.sample_id;
        output.request_id = sample.request_id;
        output.received_time_ms = sample.received_time_ms;

        return output;
    }

    filtered_global::output_snapshot rebuild_output(std::uint32_t now_ms, bool is_new_sample, bool accepted, bool rejected, const filtered_global::sample &raw_sample, std::uint16_t history_confidence)
    {
        filtered_global::sample filtered_sample = {};
        filtered_global::output_snapshot output = latest_output;

        if (filtered_global_history::newest(history, filtered_sample) == true)
        {
            output = build_output_from_sample(filtered_sample, now_ms);
        }

        output.is_new_sample = is_new_sample;
        output.accepted = accepted;
        output.rejected = rejected;
        output.raw_confidence_position = raw_sample.raw_confidence_position;
        output.history_confidence = history_confidence;

        if (is_new_sample == true)
        {
            output.sample_id = raw_sample.sample_id;
            output.request_id = raw_sample.request_id;
            output.received_time_ms = raw_sample.received_time_ms;
        }

        latest_output = output;
        return latest_output;
    }

    bool sample_is_usable(const filtered_global::sample &sample)
    {
        if (sample.valid == false)
        {
            return false;
        }

        if (sample.confidence_position < filtered_global_tuning::minimum_tracking_confidence)
        {
            return false;
        }

        return true;
    }

    filtered_global::output_snapshot update_bootstrap(const filtered_global::sample &raw_sample, std::uint32_t now_ms)
    {
        push_bootstrap_sample(raw_sample);

        filtered_global::sample initial_sample = {};
        std::uint16_t history_confidence = 0U;
        const bool has_initial_sample = bootstrap_filter::build_initial_sample(bootstrap_samples, bootstrap_sample_count, initial_sample, history_confidence);

        if (has_initial_sample == false)
        {
            return rebuild_output(now_ms, true, false, false, raw_sample, history_confidence);
        }

        initial_sample.raw_confidence_position = raw_sample.raw_confidence_position;
        filtered_global_history::push(history, initial_sample);
        reset_bootstrap_samples();
        stuck_rejected_sample_count = 0U;

        return rebuild_output(now_ms, true, true, false, raw_sample, history_confidence);
    }

    filtered_global::output_snapshot update_tracking(const filtered_global::sample &raw_sample, std::uint32_t now_ms)
    {
        const std::uint16_t history_confidence = history_filter::calculate_history_confidence(history, raw_sample);
        const std::uint16_t filtered_confidence = position_sensorfusion_internal::multiply_confidence(raw_sample.confidence_position, history_confidence);

        if (filtered_confidence < filtered_global_tuning::minimum_accepted_confidence)
        {
            if (latest_output.confidence_position == 0U)
            {
                stuck_rejected_sample_count++;

                if (stuck_rejected_sample_count >= filtered_global_tuning::stuck_recovery_rejected_sample_count)
                {
                    reset_runtime_state();
                    return update_bootstrap(raw_sample, now_ms);
                }
            }

            return rebuild_output(now_ms, true, false, true, raw_sample, history_confidence);
        }

        stuck_rejected_sample_count = 0U;

        filtered_global::sample previous_sample = {};

        if (filtered_global_history::newest(history, previous_sample) == false)
        {
            return update_bootstrap(raw_sample, now_ms);
        }

        const filtered_global::sample filtered_sample = adaptive_low_pass::update_position(previous_sample, raw_sample, filtered_confidence);
        filtered_global_history::push(history, filtered_sample);

        return rebuild_output(now_ms, true, true, false, raw_sample, history_confidence);
    }
}

namespace filtered_global
{
    void init()
    {
        reset_runtime_state();
    }

    output_snapshot update(const sample &raw_sample, std::uint32_t now_ms)
    {
        if (sample_is_usable(raw_sample) == false)
        {
            return rebuild_output(now_ms, true, false, true, raw_sample, 0U);
        }

        if (history.count == 0U)
        {
            return update_bootstrap(raw_sample, now_ms);
        }

        return update_tracking(raw_sample, now_ms);
    }

    output_snapshot read_output(std::uint32_t now_ms)
    {
        sample filtered_sample = {};

        if (filtered_global_history::newest(history, filtered_sample) == true)
        {
            latest_output = build_output_from_sample(filtered_sample, now_ms);
        }

        latest_output.is_new_sample = false;
        latest_output.accepted = false;
        latest_output.rejected = false;

        return latest_output;
    }

    std::uint8_t history_count()
    {
        return history.count;
    }

    bool read_history_sample(std::uint8_t index, sample &sample_out)
    {
        return filtered_global_history::read(history, index, sample_out);
    }
}
