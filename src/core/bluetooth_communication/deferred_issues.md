# deferred issues

This file tracks work that is intentionally left for later in `src/core/bluetooth_communication/`.

Use this file as a checklist while the Bluetooth middleware is being completed.

## done baseline

- [x] raw Bluetooth transport API exists
- [x] middleware parser selects route by category and command name
- [x] route tables exist for `inc`, `ini`, and `rsp`
- [x] pipeline connects transport, parser, route table, and handlers
- [x] handlers now own parsing of the remaining message after `(`
- [x] fake responses such as hardcoded `rsp:speed(0)` were removed from unfinished handlers
- [x] middleware handler style was cleaned up to use explicit parse steps

## finished simple payload-only handlers

- [x] `handle_ping()` parses `ping()` and writes `rsp:pong()`
- [x] `handle_dummy()` parses `dummy()` and writes `rsp:ok()`
- [x] `handle_pong()` validates `rsp:pong()`
- [x] `handle_ok()` validates `rsp:ok()`
- [x] `handle_fail()` validates `rsp:fail(uint8_t:error_code)`

## high priority

- [ ] define exact handler return contract
- [ ] decide what `true` and `false` mean at pipeline level
- [ ] decide when handlers should send `rsp:ok()` and `rsp:fail(...)`
- [ ] decide whether pipeline should react differently when a handler returns `false`

## handler backend wiring

- [ ] connect `handle_set_drive_forward_mm()` to real drive/control logic
- [ ] connect `handle_set_drive_rotate_deg()` to real drive/control logic
- [ ] connect `handle_set_speed()` to real speed/control logic
- [ ] connect `handle_set_pause_ms()` to real mission/control logic
- [ ] connect `handle_set_armed()` to real armed-state logic
- [ ] connect `handle_reset()` to real reset logic

## state-backed response handlers

- [ ] connect `handle_get_speed()` to real speed state
- [x] connect `handle_get_position_local()` to real local position state with temporary confidence mapping from `is_valid`
- [ ] revisit `handle_get_position_local()` when a real confidence source exists
- [ ] connect `handle_get_position_global()` to real global position state
- [ ] connect `handle_get_armed()` to real armed state
- [x] connect `handle_get_mission_part_current()` to minimal mission runner state

## mission handlers

- [x] define mission storage/ownership boundary for Bluetooth mission transfer
- [x] connect `handle_set_mission_new()` to mission transfer logic
- [x] connect `handle_mission_part_info()` to mission transfer logic
- [x] connect `handle_path_chunk()` to mission transfer logic
- [x] connect `handle_set_mission_start()` to minimal mission runner logic
- [x] connect `handle_set_mission_abort()` to minimal mission runner logic
- [x] connect AGV-side mission pull flow so `set_mission_new()` triggers `ini:get_mission_part(...)` and later `ini:get_path_chunk(...)`
- [ ] decide whether initiated request handler files should stay as parser stubs or be replaced by explicit sender-side modules

## watch handlers

- [ ] implement `watch_manager.cpp`
- [ ] connect `handle_set_watch_keep_alive()` to `watch_manager`
- [ ] connect `handle_set_watch_add()` to `watch_manager`
- [ ] connect `handle_set_watch_remove()` to `watch_manager`
- [ ] define how due watch commands should be emitted from the pipeline

## response-side state handling

- [ ] decide where parsed `rsp:*` values should be stored or forwarded
- [ ] connect `handle_speed()` to real response/state handling
- [ ] connect `handle_armed()` to real response/state handling
- [ ] connect `handle_mission_part_current()` to real response/state handling
- [ ] connect `handle_ok()` and `handle_fail()` to real request/ack handling if needed

## parser and pipeline follow-up

- [ ] verify parser behavior with partial messages split across multiple ticks
- [ ] verify parser behavior with malformed category text
- [ ] verify parser behavior with unknown command names
- [ ] verify timeout behavior in handlers under slow byte arrival
- [ ] add timeout or retry handling for mission pull requests waiting on `rsp:ok()` or `rsp:fail(...)`
- [ ] decide whether pipeline should log or count parser errors
- [ ] decide whether the pipeline should process initiated watch output in the same tick or separately

## tests and verification

- [ ] add parser-focused tests or a small host-side harness
- [ ] add handler parsing tests for simple commands
- [ ] add handler parsing tests for mission transfer commands
- [ ] test binary `path_chunk` parsing with realistic payloads
- [ ] run full compile verification once the separate board/header issue is resolved

## separate blocker outside this folder

- [ ] fix the existing board/header issue that blocked compile verification
