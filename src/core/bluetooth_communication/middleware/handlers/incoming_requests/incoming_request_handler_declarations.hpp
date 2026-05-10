#pragma once

namespace incoming_request_handlers
{
    bool handle_ping();
    bool handle_reset();
    bool handle_get_speed();
    bool handle_get_position_local();
    bool handle_get_position_global();
    bool handle_set_position_local_reset();
    bool handle_set_armed();
    bool handle_set_obstacle_safety();
    bool handle_get_armed();
    bool handle_set_drive_forward_mm();
    bool handle_set_mission_new();
    bool handle_mission_part_info();
    bool handle_path_chunk();
    bool handle_set_mission_start();
    bool handle_set_mission_abort();
    bool handle_set_pause_ms();
    bool handle_set_speed();
    bool handle_get_mission_part_current();
    bool handle_set_drive_rotate_deg();
    bool handle_set_watch_keep_alive();
    bool handle_set_watch_add();
    bool handle_set_watch_remove();
    bool handle_set_imu_calibration_start();
    bool handle_set_imu_calibration_clear();
    bool handle_dummy();
}
