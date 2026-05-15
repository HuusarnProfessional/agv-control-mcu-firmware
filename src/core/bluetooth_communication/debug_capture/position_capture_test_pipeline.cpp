#include "position_capture_test_pipeline.hpp"

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <cstdio>

#include "../bluetooth_transport.hpp"
#include "../../control/primitives/command_speed/command_speed_state.hpp"
#include "../../control/primitives/motion_primitive/motion_primitive_status_monitor.hpp"
#include "../../global_positioning/global_position_api.hpp"
#include "../../motion_mcu_communication/outgoing_payloads/service/drive_forward_payload.hpp"
#include "../../motion_mcu_communication/state/incoming/incoming_state.hpp"
#include "../../position_sensorfusion/filtered_global_position/filtered_global_position.hpp"

namespace
{
    constexpr std::int32_t test_distance_mm = 1000;
    constexpr std::int64_t test_distance_um = 1000000LL;
    constexpr std::size_t max_local_record_count = 700U;
    constexpr std::size_t max_global_filter_record_count = 96U;

    constexpr std::uint8_t local_has_pose_flag = 0x01U;
    constexpr std::uint8_t global_valid_flag = 0x01U;
    constexpr std::uint8_t filter_has_position_flag = 0x02U;
    constexpr std::uint8_t filter_has_heading_flag = 0x04U;
    constexpr std::uint8_t filter_is_new_sample_flag = 0x08U;
    constexpr std::uint8_t filter_rejected_flag = 0x10U;
    constexpr std::uint8_t filter_accepted_flag = 0x20U;

    enum class capture_state : std::uint8_t
    {
        idle = 0U,
        start_requested,
        capturing,
        streaming_header,
        streaming_local_columns,
        streaming_local_records,
        streaming_global_filter_columns,
        streaming_global_filter_records,
        streaming_footer
    };

    struct local_tick_record
    {
        std::uint32_t now_ms = 0U;
        std::int32_t x_um = 0;
        std::int32_t y_um = 0;
        std::int32_t heading_urad = 0;
        std::uint16_t confidence_position = 0U;
        std::uint16_t confidence_heading = 0U;
        std::uint8_t pose_id = 0U;
        std::uint8_t branch_id = 0U;
        std::uint8_t flags = 0U;
    };

    struct global_filter_record
    {
        std::uint32_t now_ms = 0U;
        std::uint32_t global_sample_id = 0U;
        std::uint32_t global_received_time_ms = 0U;
        std::uint32_t filter_sample_id = 0U;
        std::uint32_t filter_received_time_ms = 0U;
        std::int32_t global_x_mm = 0;
        std::int32_t global_y_mm = 0;
        std::int32_t global_z_mm = 0;
        std::int32_t filter_x_um = 0;
        std::int32_t filter_y_um = 0;
        std::int32_t filter_z_um = 0;
        std::int32_t filter_heading_urad = 0;
        std::uint16_t filter_raw_confidence_position = 0U;
        std::uint16_t filter_history_confidence = 0U;
        std::uint16_t filter_confidence_position = 0U;
        std::uint16_t filter_confidence_heading = 0U;
        std::uint8_t filter_accepted_sample_count = 0U;
        std::uint8_t filter_heading_sample_count = 0U;
        std::uint8_t global_quality_factor = 0U;
        std::uint8_t global_status = 0U;
        std::uint8_t flags = 0U;
    };

    capture_state g_state = capture_state::idle;
    local_tick_record g_local_records[max_local_record_count] = {};
    global_filter_record g_global_filter_records[max_global_filter_record_count] = {};
    std::size_t g_local_record_count = 0U;
    std::size_t g_global_filter_record_count = 0U;
    bool g_local_overflow = false;
    bool g_global_filter_overflow = false;
    std::uint32_t g_start_time_ms = 0U;
    std::uint32_t g_complete_time_ms = 0U;
    std::uint16_t g_speed_mm_s = 0U;
    bool g_success = false;
    bool g_timed_out = false;
    std::size_t g_stream_index = 0U;
    bool g_has_last_local_record = false;
    bool g_has_last_global_filter_record = false;
    local_tick_record g_last_local_record = {};
    global_filter_record g_last_global_filter_record = {};

    bool write_line(const char *text)
    {
        if (text == nullptr)
        {
            return false;
        }

        const std::size_t text_length = std::strlen(text);
        static constexpr char line_ending[] = "\n";

        if (bluetooth_transport::write_bytes(reinterpret_cast<const std::uint8_t *>(text), text_length) != bluetooth_transport::transport_status::ok)
        {
            return false;
        }

        if (bluetooth_transport::write_bytes(reinterpret_cast<const std::uint8_t *>(line_ending), sizeof(line_ending) - 1U) != bluetooth_transport::transport_status::ok)
        {
            return false;
        }

        return true;
    }

    void clear_capture_state()
    {
        g_local_record_count = 0U;
        g_global_filter_record_count = 0U;
        g_local_overflow = false;
        g_global_filter_overflow = false;
        g_start_time_ms = 0U;
        g_complete_time_ms = 0U;
        g_speed_mm_s = 0U;
        g_success = false;
        g_timed_out = false;
        g_stream_index = 0U;
        g_has_last_local_record = false;
        g_has_last_global_filter_record = false;
        g_last_local_record = {};
        g_last_global_filter_record = {};
    }

    void finish_and_prepare_stream(std::uint32_t now_ms)
    {
        const motion_primitive_status_monitor::snapshot primitive_status = motion_primitive_status_monitor::read_snapshot();

        g_complete_time_ms = now_ms;
        g_success = primitive_status.success;
        g_timed_out = primitive_status.timed_out;
        g_stream_index = 0U;
        g_state = capture_state::streaming_header;
    }

    bool capture_storage_is_full()
    {
        if (g_local_overflow == true)
        {
            return true;
        }

        if (g_global_filter_overflow == true)
        {
            return true;
        }

        return false;
    }

    void append_local_record(const local_tick_record &record)
    {
        if (g_local_record_count >= max_local_record_count)
        {
            g_local_overflow = true;
            return;
        }

        g_local_records[g_local_record_count] = record;
        g_local_record_count++;
    }

    void append_global_filter_record(const global_filter_record &record)
    {
        if (g_global_filter_record_count >= max_global_filter_record_count)
        {
            g_global_filter_overflow = true;
            return;
        }

        g_global_filter_records[g_global_filter_record_count] = record;
        g_global_filter_record_count++;
    }

    local_tick_record build_local_record(std::uint32_t now_ms)
    {
        local_tick_record record = {};
        const motion_mcu_incoming_state::local_position_state local_position = motion_mcu_incoming_state::get_local_position();

        record.now_ms = now_ms;
        record.x_um = static_cast<std::int32_t>(local_position.x_um);
        record.y_um = static_cast<std::int32_t>(local_position.y_um);
        record.heading_urad = local_position.heading_urad;
        record.confidence_position = local_position.confidence_position;
        record.confidence_heading = local_position.confidence_heading;
        record.pose_id = local_position.pose_id;
        record.branch_id = local_position.branch_id;

        if (local_position.has_pose == true)
        {
            record.flags |= local_has_pose_flag;
        }

        return record;
    }

    global_filter_record build_global_filter_record(std::uint32_t now_ms)
    {
        global_filter_record record = {};
        global_position_api::global_position_sample global_sample = {};
        const bool has_global_sample = global_position_api::read_sample(global_sample);
        const filtered_global_position::output_snapshot filter_output = filtered_global_position::read_output(now_ms);

        record.now_ms = now_ms;
        record.global_sample_id = global_sample.sample_id;
        record.global_received_time_ms = global_sample.received_time_ms;
        record.filter_sample_id = filter_output.sample_id;
        record.filter_received_time_ms = filter_output.received_time_ms;
        record.global_x_mm = global_sample.x_mm;
        record.global_y_mm = global_sample.y_mm;
        record.global_z_mm = global_sample.z_mm;
        record.filter_x_um = static_cast<std::int32_t>(filter_output.x_um);
        record.filter_y_um = static_cast<std::int32_t>(filter_output.y_um);
        record.filter_z_um = static_cast<std::int32_t>(filter_output.z_um);
        record.filter_heading_urad = filter_output.heading_urad;
        record.filter_raw_confidence_position = filter_output.raw_confidence_position;
        record.filter_history_confidence = filter_output.history_confidence;
        record.filter_confidence_position = filter_output.confidence_position;
        record.filter_confidence_heading = filter_output.confidence_heading;
        record.filter_accepted_sample_count = filter_output.accepted_sample_count;
        record.filter_heading_sample_count = filter_output.heading_sample_count;
        record.global_quality_factor = global_sample.quality_factor;
        record.global_status = static_cast<std::uint8_t>(global_sample.status);

        if (has_global_sample == true)
        {
            record.flags |= global_valid_flag;
        }

        if (filter_output.has_position == true)
        {
            record.flags |= filter_has_position_flag;
        }

        if (filter_output.has_heading == true)
        {
            record.flags |= filter_has_heading_flag;
        }

        if (filter_output.is_new_sample == true)
        {
            record.flags |= filter_is_new_sample_flag;
        }

        if (filter_output.rejected == true)
        {
            record.flags |= filter_rejected_flag;
        }

        if (filter_output.accepted == true)
        {
            record.flags |= filter_accepted_flag;
        }

        return record;
    }

    bool local_record_changed(const local_tick_record &record)
    {
        if (g_has_last_local_record == false)
        {
            return true;
        }

        if (record.flags != g_last_local_record.flags)
        {
            return true;
        }

        if (record.x_um != g_last_local_record.x_um)
        {
            return true;
        }

        if (record.y_um != g_last_local_record.y_um)
        {
            return true;
        }

        if (record.heading_urad != g_last_local_record.heading_urad)
        {
            return true;
        }

        if (record.confidence_position != g_last_local_record.confidence_position)
        {
            return true;
        }

        if (record.confidence_heading != g_last_local_record.confidence_heading)
        {
            return true;
        }

        if (record.pose_id != g_last_local_record.pose_id)
        {
            return true;
        }

        if (record.branch_id != g_last_local_record.branch_id)
        {
            return true;
        }

        return false;
    }

    bool global_filter_record_changed(const global_filter_record &record)
    {
        if (g_has_last_global_filter_record == false)
        {
            return true;
        }

        if (record.flags != g_last_global_filter_record.flags)
        {
            return true;
        }

        if (record.global_sample_id != g_last_global_filter_record.global_sample_id)
        {
            return true;
        }

        if (record.filter_sample_id != g_last_global_filter_record.filter_sample_id)
        {
            return true;
        }

        if (record.global_received_time_ms != g_last_global_filter_record.global_received_time_ms)
        {
            return true;
        }

        if (record.filter_received_time_ms != g_last_global_filter_record.filter_received_time_ms)
        {
            return true;
        }

        return false;
    }

    void capture_local_if_changed(std::uint32_t now_ms)
    {
        const local_tick_record record = build_local_record(now_ms);

        if (local_record_changed(record) == false)
        {
            return;
        }

        append_local_record(record);
        g_last_local_record = record;
        g_has_last_local_record = true;
    }

    void capture_global_filter_if_changed(std::uint32_t now_ms)
    {
        const global_filter_record record = build_global_filter_record(now_ms);

        if (global_filter_record_changed(record) == false)
        {
            return;
        }

        append_global_filter_record(record);
        g_last_global_filter_record = record;
        g_has_last_global_filter_record = true;
    }

    void capture_changed_records(std::uint32_t now_ms)
    {
        capture_local_if_changed(now_ms);
        capture_global_filter_if_changed(now_ms);
    }

    bool send_drive_forward(std::uint32_t now_ms)
    {
        g_speed_mm_s = command_speed_state::get_requested_speed_mm_s();

        if (g_speed_mm_s == 0U)
        {
            write_line("position_capture_test error speed_zero");
            g_state = capture_state::idle;
            return false;
        }

        if (drive_forward_payload::send(static_cast<std::int32_t>(g_speed_mm_s), test_distance_um) == false)
        {
            write_line("position_capture_test error send_failed");
            g_state = capture_state::idle;
            return false;
        }

        motion_primitive_status_monitor::notify_drive_forward_sent(now_ms);
        g_start_time_ms = now_ms;
        g_state = capture_state::capturing;
        capture_changed_records(now_ms);

        return true;
    }

    bool stream_header()
    {
        char summary[224] = {};

        if (write_line("position_capture_test streaming") == false)
        {
            return false;
        }

        std::snprintf(summary, sizeof(summary), "position_capture_summary distance_mm %ld speed_mm_s %lu local_records %lu local_overflow %u global_filter_records %lu global_filter_overflow %u success %u timed_out %u start_time_ms %lu complete_time_ms %lu", static_cast<long>(test_distance_mm), static_cast<unsigned long>(g_speed_mm_s), static_cast<unsigned long>(g_local_record_count), g_local_overflow ? 1U : 0U, static_cast<unsigned long>(g_global_filter_record_count), g_global_filter_overflow ? 1U : 0U, g_success ? 1U : 0U, g_timed_out ? 1U : 0U, static_cast<unsigned long>(g_start_time_ms), static_cast<unsigned long>(g_complete_time_ms));

        if (write_line(summary) == false)
        {
            return false;
        }

        g_state = capture_state::streaming_local_columns;

        return true;
    }

    bool stream_local_columns()
    {
        if (write_line("position_capture_local_columns idx now_ms has_pose x_um y_um heading_urad confidence_position confidence_heading pose_id branch_id") == false)
        {
            return false;
        }

        g_stream_index = 0U;
        g_state = capture_state::streaming_local_records;

        return true;
    }

    bool stream_local_record()
    {
        char line[256] = {};

        if (g_stream_index >= g_local_record_count)
        {
            g_state = capture_state::streaming_global_filter_columns;
            return true;
        }

        const local_tick_record &record = g_local_records[g_stream_index];
        std::snprintf(line, sizeof(line), "position_capture_local_record %lu %lu %u %ld %ld %ld %u %u %u %u", static_cast<unsigned long>(g_stream_index), static_cast<unsigned long>(record.now_ms), (record.flags & local_has_pose_flag) != 0U ? 1U : 0U, static_cast<long>(record.x_um), static_cast<long>(record.y_um), static_cast<long>(record.heading_urad), static_cast<unsigned>(record.confidence_position), static_cast<unsigned>(record.confidence_heading), static_cast<unsigned>(record.pose_id), static_cast<unsigned>(record.branch_id));

        if (write_line(line) == false)
        {
            return false;
        }

        g_stream_index++;

        return true;
    }

    bool stream_global_filter_columns()
    {
        if (write_line("position_capture_global_filter_columns idx now_ms global_valid global_sample_id global_received_time_ms global_x_mm global_y_mm global_z_mm global_quality_factor global_status filter_has_position filter_x_um filter_y_um filter_z_um filter_raw_confidence_position filter_history_confidence filter_confidence_position filter_has_heading filter_heading_urad filter_confidence_heading filter_is_new_sample filter_accepted filter_rejected filter_sample_id filter_received_time_ms filter_accepted_sample_count filter_heading_sample_count") == false)
        {
            return false;
        }

        g_stream_index = 0U;
        g_state = capture_state::streaming_global_filter_records;

        return true;
    }

    bool stream_global_filter_record()
    {
        char line[512] = {};

        if (g_stream_index >= g_global_filter_record_count)
        {
            g_state = capture_state::streaming_footer;
            return true;
        }

        const global_filter_record &record = g_global_filter_records[g_stream_index];
        std::snprintf(line, sizeof(line), "position_capture_global_filter_record %lu %lu %u %lu %lu %ld %ld %ld %u %u %u %ld %ld %ld %u %u %u %u %ld %u %u %u %u %lu %lu %u %u", static_cast<unsigned long>(g_stream_index), static_cast<unsigned long>(record.now_ms), (record.flags & global_valid_flag) != 0U ? 1U : 0U, static_cast<unsigned long>(record.global_sample_id), static_cast<unsigned long>(record.global_received_time_ms), static_cast<long>(record.global_x_mm), static_cast<long>(record.global_y_mm), static_cast<long>(record.global_z_mm), static_cast<unsigned>(record.global_quality_factor), static_cast<unsigned>(record.global_status), (record.flags & filter_has_position_flag) != 0U ? 1U : 0U, static_cast<long>(record.filter_x_um), static_cast<long>(record.filter_y_um), static_cast<long>(record.filter_z_um), static_cast<unsigned>(record.filter_raw_confidence_position), static_cast<unsigned>(record.filter_history_confidence), static_cast<unsigned>(record.filter_confidence_position), (record.flags & filter_has_heading_flag) != 0U ? 1U : 0U, static_cast<long>(record.filter_heading_urad), static_cast<unsigned>(record.filter_confidence_heading), (record.flags & filter_is_new_sample_flag) != 0U ? 1U : 0U, (record.flags & filter_accepted_flag) != 0U ? 1U : 0U, (record.flags & filter_rejected_flag) != 0U ? 1U : 0U, static_cast<unsigned long>(record.filter_sample_id), static_cast<unsigned long>(record.filter_received_time_ms), static_cast<unsigned>(record.filter_accepted_sample_count), static_cast<unsigned>(record.filter_heading_sample_count));

        if (write_line(line) == false)
        {
            return false;
        }

        g_stream_index++;

        return true;
    }

    bool stream_footer()
    {
        char line[192] = {};

        std::snprintf(line, sizeof(line), "position_capture_test done local_records %lu local_overflow %u global_filter_records %lu global_filter_overflow %u success %u timed_out %u", static_cast<unsigned long>(g_local_record_count), g_local_overflow ? 1U : 0U, static_cast<unsigned long>(g_global_filter_record_count), g_global_filter_overflow ? 1U : 0U, g_success ? 1U : 0U, g_timed_out ? 1U : 0U);

        if (write_line(line) == false)
        {
            return false;
        }

        g_state = capture_state::idle;

        return true;
    }
}

namespace position_capture_test_pipeline
{
    void init()
    {
        g_state = capture_state::idle;
        clear_capture_state();
    }

    void tick(std::uint32_t now_ms)
    {
        if (g_state == capture_state::idle)
        {
            return;
        }

        if (g_state == capture_state::start_requested)
        {
            clear_capture_state();
            send_drive_forward(now_ms);
            return;
        }

        if (g_state == capture_state::capturing)
        {
            capture_changed_records(now_ms);

            if (capture_storage_is_full() == true)
            {
                finish_and_prepare_stream(now_ms);
                return;
            }

            if (motion_primitive_status_monitor::was_successful() == true)
            {
                capture_changed_records(now_ms);
                finish_and_prepare_stream(now_ms);
            }

            return;
        }

        if (g_state == capture_state::streaming_header)
        {
            stream_header();
            return;
        }

        if (g_state == capture_state::streaming_local_columns)
        {
            stream_local_columns();
            return;
        }

        if (g_state == capture_state::streaming_local_records)
        {
            stream_local_record();
            return;
        }

        if (g_state == capture_state::streaming_global_filter_columns)
        {
            stream_global_filter_columns();
            return;
        }

        if (g_state == capture_state::streaming_global_filter_records)
        {
            stream_global_filter_record();
            return;
        }

        if (g_state == capture_state::streaming_footer)
        {
            stream_footer();
            return;
        }
    }

    bool request_run()
    {
        if (g_state != capture_state::idle)
        {
            return false;
        }

        if (motion_primitive_status_monitor::is_waiting() == true)
        {
            return false;
        }

        g_state = capture_state::start_requested;

        return true;
    }
}
