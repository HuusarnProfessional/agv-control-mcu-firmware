# deferred issues

This file lists only the work that is still missing on the ESP side.

Completed work is intentionally omitted.

## bluetooth handlers that are still parser stubs

These handlers parse their arguments today, but still end in `return false;` and do not perform real work.

- [ ] `incoming_request_handlers::handle_reset()`
- [ ] `incoming_request_handlers::handle_get_position_global()`
- [ ] `incoming_request_handlers::handle_get_armed()`

## mission execution is still structurally incomplete

- [ ] decide how `set_drive_rotate_deg(...)` should interact with path-following between mission parts
- [ ] decide whether the current Java mission segmentation is acceptable for path-following

## control pipelines that are still empty

- [ ] implement `src/core/global_positioning/global_positioning_pipeline.cpp`

## bluetooth state and control backends still missing

- [ ] define real armed-state ownership on ESP
- [ ] define real speed-state ownership on ESP
- [ ] define real global-position ownership on ESP
- [ ] implement real reset behavior for `reset()`

## motion mcu integration still missing at the upper layer

The STM payload senders now exist, but the Bluetooth/control layer is still not using them.

- [ ] decide how armed-state should affect outgoing STM motion commands
- [ ] decide whether `pause_payload` should remain unused on ESP because pause now lives locally
- [ ] decide whether `trailer_status`, `obstacle_safety_control`, and similar STM service payloads are needed now or can stay deferred
- [ ] revisit the temporary rotation-speed mapping used by `set_drive_rotate_deg(...)` if `set_speed(...)` later gets stricter semantics

## stm heartbeat follow-up

- [ ] decide what control behavior should happen when `motion_mcu_heartbeat` reports timeout
- [ ] decide whether Bluetooth-visible commands should fail differently when the STM link is timed out

## protocol alignment still open

- [ ] lock the official mission command vocabulary with the Java/KTS side
- [ ] decide whether unused `rsp:*` routes should remain in the route table or be removed until they have a real consumer

## initiated request handlers that are still parser stubs

These files are not used in the current mission transfer flow. AGV-side mission pull currently goes through `mission_transfer` and direct outgoing `ini:*` text requests from the pipeline.

If these files stay, they still only parse incoming initiated requests and then fail.

- [ ] `initiated_request_handlers::handle_get_mission_part()`
- [ ] `initiated_request_handlers::handle_get_path_chunk()`

## rsp handlers that still have no storage or consumer

These are `rsp:*` handlers. They parse correctly today, but the parsed values are not stored anywhere useful yet.

- [ ] `response_handlers::handle_speed()`
- [ ] `response_handlers::handle_armed()`
- [ ] `response_handlers::handle_mission_part_current()`

## watch handlers that are still parser stubs

- [ ] `incoming_request_handlers::handle_set_watch_keep_alive()`
- [ ] `incoming_request_handlers::handle_set_watch_add()`
- [ ] `incoming_request_handlers::handle_set_watch_remove()`

## watch subsystem still missing

- [ ] implement `src/core/bluetooth_communication/middleware/watch/watch_manager.cpp`
- [ ] initialize `watch_manager` from the Bluetooth side
- [ ] tick `watch_manager` from the Bluetooth side
- [ ] connect `handle_set_watch_keep_alive()` to `watch_manager`
- [ ] connect `handle_set_watch_add()` to `watch_manager`
- [ ] connect `handle_set_watch_remove()` to `watch_manager`
- [ ] emit due watch commands from `bluetooth_communication_pipeline.cpp`
